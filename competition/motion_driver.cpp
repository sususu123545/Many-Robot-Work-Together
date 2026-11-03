// High-level L1 SDK adapter. It never writes joint targets or replaces firmware.
#include "zsibot_sdk/zsibot_api.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>

using Clock=std::chrono::steady_clock;
volatile std::sig_atomic_t running=1;
void stop_signal(int){running=0;}
std::array<float,4> limited(float forward,float yaw,float lateral,bool fresh){
    if(!fresh||!std::isfinite(forward)||!std::isfinite(yaw)||!std::isfinite(lateral))return {};
    return {std::clamp(forward,-.3f,.3f),std::clamp(yaw,-.3f,.3f),std::clamp(lateral,-.3f,.3f),0};
}
int main(int argc,char**argv){
    std::string ip;unsigned send_port=8081,recv_port=8080;
    for(int i=1;i<argc;i++){
        std::string arg=argv[i];
        if(arg=="--help"){
            std::cout<<"Usage: zsl1_motion --robot-ip IP [--send-port 8081] [--recv-port 8080]\n"
              <<"STDIN: claim | stand | walk | balance | move FORWARD YAW LATERAL | stop | estop | crawl | climb | lab_on | jump | forward_jump | quit\n"
              <<"Move values are normalized inputs, not m/s. Default state is unclaimed.\n";return 0;
        }
        if(arg=="--self-test"){
            bool passed=limited(2,-2,.1f,true)[0]==.3f&&limited(2,-2,.1f,true)[1]==-.3f
                &&limited(.2f,.1f,0,false)==std::array<float,4>{}&&limited(NAN,0,0,true)==std::array<float,4>{};
            if(!passed){std::cerr<<"Command-limit test failed\n";return 1;}
            std::cout<<"PASS: input limits, stale-command zeroing, nonfinite-input rejection\n";return 0;
        }
        if(arg=="--robot-ip"&&i+1<argc)ip=argv[++i];
        else if(arg=="--send-port"&&i+1<argc)send_port=std::stoul(argv[++i]);
        else if(arg=="--recv-port"&&i+1<argc)recv_port=std::stoul(argv[++i]);
        else{std::cerr<<"Unknown or incomplete option: "<<arg<<'\n';return 2;}
    }
    if(ip.empty()||send_port>65535||recv_port>65535||send_port==0||recv_port==0){std::cerr<<"Explicit robot IP and valid UDP ports are required\n";return 2;}
    std::signal(SIGTERM,stop_signal);std::signal(SIGINT,stop_signal);
    try{
        zsibot::ZsibotExecutor robot(zsibot::Role::ROLE_SDK,ip,send_port,recv_port);
        bool claimed=false,lab=false;std::array<float,4> desired{};std::array<float,14> buttons{};
        auto special_pause_until=Clock::now();
        auto last_move=Clock::now()-std::chrono::seconds(10);auto last_status=last_move;
        std::string buffer;
        while(running){
            pollfd input{STDIN_FILENO,POLLIN,0};int ready=poll(&input,1,0);
            if(ready>0&&(input.revents&(POLLIN|POLLHUP))){
                char chunk[1024];auto count=read(STDIN_FILENO,chunk,sizeof(chunk));
                if(count<=0){running=0;break;}
                buffer.append(chunk,static_cast<size_t>(count));
                if(buffer.size()>4096){buffer.clear();desired={};std::cerr<<"Input line too long; stopped\n";}
                size_t end;
                while((end=buffer.find('\n'))!=std::string::npos){
                    std::string line=buffer.substr(0,end);buffer.erase(0,end+1);
                    std::istringstream command(line);std::string name;command>>name;
                    if(name=="quit"){running=0;break;}
                    if(name=="stop"){desired={};last_move=Clock::now();continue;}
                    if(!robot.IsConnected()){std::cerr<<"Robot is not connected; command rejected\n";continue;}
                    if(name=="estop"){robot.SetRemote({},buttons);robot.SetCmd(zsibot::CmdCode::CMD_EMERGENCY_STOP);claimed=false;desired={};continue;}
                    if(name=="claim"){robot.SetCmd(zsibot::CmdCode::CMD_GENERAL_SDK_CONTROL_RIGHT);claimed=true;desired={};continue;}
                    if(!claimed){std::cerr<<"Control has not been claimed\n";continue;}
                    using C=zsibot::CmdCode;
                    if(name=="move"){
                        float forward=0,yaw=0,lateral=0;
                        if(!(command>>forward>>yaw>>lateral)){desired={};std::cerr<<"Invalid move command\n";continue;}
                        if(robot.GetFunctionMode()!=zsibot::FunctionMode::FM_GENERAL_SDK||robot.GetControlMode()!=zsibot::ControlMode::CM_MOVE_MODE){desired={};std::cerr<<"SDK/move mode not confirmed by firmware\n";continue;}
                        desired=limited(forward,yaw,lateral,true);last_move=Clock::now();
                    }else{
                        desired={};robot.SetRemote({},buttons);
                        if(name=="stand")robot.SetCmd(C::CMD_STAND_UP);
                        else if(name=="walk")robot.SetCmd(C::CMD_MOVE_MODE);
                        else if(name=="balance")robot.SetCmd(C::CMD_BALANCE_STAND_MODE);
                        else if(name=="slow")robot.SetCmd(C::CMD_SLOW_SPEED);
                        else if(name=="normal")robot.SetCmd(C::CMD_NORMAL_SPEED);
                        else if(name=="fast")robot.SetCmd(C::CMD_FAST_SPEED);
                        else if(name=="crawl")robot.SetCmd(C::CMD_CRAWL_FORWARD);
                        else if(name=="climb")robot.SetCmd(C::CMD_CLIMBING_HIGH_PLATFORM);
                        else if(name=="lab_on"){robot.SetCmd(C::CMD_ENTER_LAB_MODE);lab=true;}
                        else if(name=="jump"||name=="forward_jump"){
                            if(lab&&robot.GetModel()==zsibot::Model::MODEL_XG&&robot.GetMotionMode()==zsibot::MotionMode::MM_REST){
                                robot.SetCmd(name=="jump"?C::CMD_JUMP:C::CMD_FORWARD_JUMP);special_pause_until=Clock::now()+std::chrono::seconds(2);
                            }
                            else std::cerr<<"Jump prerequisites are not confirmed by firmware\n";
                        }else std::cerr<<"Unknown command: "<<name<<'\n';
                    }
                }
            }
            auto now=Clock::now();bool fresh=std::chrono::duration<double>(now-last_move).count()<.25;
            if(claimed&&robot.IsConnected()&&now>=special_pause_until&&robot.GetMotionMode()!=zsibot::MotionMode::MM_RUNNING)
                robot.SetRemote(limited(desired[0],desired[1],desired[2],fresh),buttons);
            if(now-last_status>std::chrono::seconds(1)){
                auto version=robot.GetVersion();auto rpy=robot.GetRPY();auto battery=robot.GetBatteryInfo();
                std::cout<<"{\"connected\":"<<std::boolalpha<<robot.IsConnected()<<",\"claimed\":"<<claimed
                  <<",\"model\":"<<static_cast<int>(robot.GetModel())<<",\"mc_version\":"<<std::quoted(version.mc_version)
                  <<",\"task_version\":"<<std::quoted(version.dog_task_version)<<",\"control_mode\":"<<static_cast<int>(robot.GetControlMode())
                  <<",\"battery_percent\":"<<battery.power<<",\"battery_error\":"<<battery.error
                  <<",\"rpy\":["<<rpy[0]<<','<<rpy[1]<<','<<rpy[2]<<"]}"<<std::endl;last_status=now;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if(claimed&&robot.IsConnected())robot.SetRemote({},buttons);
    }catch(const std::exception&error){std::cerr<<error.what()<<'\n';return 1;}
    return 0;
}

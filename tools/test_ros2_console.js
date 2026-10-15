// DOM/socket doubles ONLY: never opens a browser or a real network connection.
'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const html=fs.readFileSync('web_console/ros2_console.html','utf8');
const source=[...html.matchAll(/<script\b[^>]*>([\s\S]*?)<\/script>/gi)].map(m=>m[1]).join('\n');
const elements=new Map(),calls=[];let connections=0;
function element(id){if(!elements.has(id))elements.set(id,{id,value:'',textContent:'',disabled:false,checked:false,dataset:{},addEventListener(){},appendChild(){}});return elements.get(id);}
element('endpoint').value='ws://127.0.0.1:9090';
const names=['joint1','joint2','joint3','joint4','joint5','r_joint'];
const jointButtons=names.map(name=>({dataset:{joint:name},disabled:false}));
const baseButtons=[{dataset:{velocity:'0.025,0,0'},addEventListener(){},disabled:false}];
class Socket{static OPEN=1;constructor(){connections++;this.readyState=1;Socket.last=this;}send(m){calls.push(JSON.parse(m));}close(){this.readyState=3;if(this.onclose)this.onclose();}}
const ctx=vm.createContext({document:{getElementById:element,createElement:()=>element('row'+elements.size),querySelectorAll:s=>s==='[data-joint]'?jointButtons:baseButtons,addEventListener(){}},window:{addEventListener(){}},WebSocket:Socket,performance:{now:()=>1000},setInterval:()=>1,clearInterval(){},console,Date});
vm.runInContext(source,ctx);assert.equal(connections,0,'Loading must not auto-connect or authorize');
element('connect').onclick();Socket.last.onopen();
assert(calls.every(c=>c.op==='subscribe'),'Connection may only subscribe');
function frame(topic,status){vm.runInContext('receive('+JSON.stringify({data:JSON.stringify({op:'publish',topic,msg:{data:JSON.stringify(status)}})})+')',ctx);}
let arm={dry_run:false,estop:false,fault:null,current_read_error:null,health_state:'READY',healthy_seconds:10,reset_eligible:true,feedback_real:true,feedback_complete:true,enable_arm_motion:false,enable_gripper_motion:false,max_step_ticks:100,stamp:{sec:100,nanosec:0},joints:Object.fromEntries(names.map(n=>[n,{raw:n==='r_joint'?86:500,temperature:31}])),feedback_age_seconds:Object.fromEntries(names.map(n=>[n,.2])),transport:{counters:{requests:12}}};
frame('/arm_controller/status',arm);frame('/chassis_control/status',{dry_run:false,enable_motion:false,estop:false,hardware_fault:false});
assert(element('authorizeGripper').disabled,'Checkbox must not be auto-checked');
element('permit').checked=true;element('permit').onchange();
const beforeGrant=calls.length;element('authorizeGripper').onclick();
const grant=calls.slice(beforeGrant).find(c=>c.service==='/arm_controller/set_parameters_atomically');
assert(grant);assert.deepEqual(grant.args.parameters.map(p=>[p.name,p.value.bool_value]),[['enable_arm_motion',false],['enable_gripper_motion',true]]);
assert(!vm.runInContext("ready('gripper')",ctx),'Wait for backend status; do not assume grant succeeded');
arm={...arm,enable_gripper_motion:true};frame('/arm_controller/status',arm);
assert(vm.runInContext("ready('gripper')",ctx));assert(!vm.runInContext("ready('arm')",ctx));
assert(jointButtons.slice(0,5).every(b=>b.disabled));assert(!jointButtons[5].disabled);
arm={...arm,feedback_complete:false};frame('/arm_controller/status',arm);
assert(!vm.runInContext("ready('gripper')",ctx));assert(element('authorizeGripper').disabled);
arm={...arm,feedback_complete:true,feedback_age_seconds:{...arm.feedback_age_seconds,joint1:1.5}};frame('/arm_controller/status',arm);
assert(!vm.runInContext("ready('gripper')",ctx),'Expired individual feedback must block commands');
arm={...arm,feedback_age_seconds:Object.fromEntries(names.map(n=>[n,.2])),estop:true,health_state:'RECOVERED_AWAIT_RESET'};
const beforeRecovery=calls.length;frame('/arm_controller/status',arm);
assert.equal(calls.length,beforeRecovery,'Recovery must not auto-reset or reissue targets');
assert(!element('reset').disabled);assert(element('authorizeGripper').disabled);
element('reset').onclick();assert(!element('permit').checked);
const resetCalls=calls.slice(beforeRecovery).filter(c=>c.op==='call_service');
assert.deepEqual(resetCalls.map(c=>c.service),['/arm_controller/reset_estop']);
assert(!calls.some(c=>c.op==='publish'&&/controller\/command$/.test(c.topic)),'Test must not accidentally publish a movement');
console.log('ROS2 console mock checks passed: explicit grant, complete freshness, isolated gripper, no auto-reset/motion');

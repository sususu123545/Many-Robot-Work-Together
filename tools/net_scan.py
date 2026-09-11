# -*- coding: utf-8 -*-
import subprocess, re, socket, concurrent.futures, json

def run(cmd):
    p = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                       encoding="gbk", errors="replace")
    return p.stdout or ""

# ---- 本机 IPv4 ----
cfg = run("ipconfig")
print("===== 本机网卡 IPv4 =====")
cur = ""
for line in cfg.splitlines():
    if line and not line.startswith(" ") and ":" in line:
        cur = line.strip()
    m = re.search(r"IPv4.*?:\s*([0-9.]+)", line)
    if m:
        print("  %-46s %s" % (cur, m.group(1)))
    m2 = re.search(r"(子网掩码|Subnet Mask).*?:\s*([0-9.]+)", line)
    if m2:
        print("      mask:", m2.group(2))

# ---- 常见网段扫 ARP ----
def ping(ip):
    subprocess.run("ping -n 1 -w 250 " + ip, shell=True,
                   capture_output=True)
    return ip

locals_ = socket.gethostbyname_ex(socket.gethostname())[2]
nets = set()
for ip in locals_:
    nets.add(".".join(ip.split(".")[:3]))
for extra in ("10.120.150", "192.168.43"):
    nets.add(extra)

print("\n===== 网段:", ", ".join(sorted(nets)))
targets = ["%s.%d" % (n, i) for n in sorted(nets) for i in range(1, 255)]
with concurrent.futures.ThreadPoolExecutor(max_workers=64) as ex:
    list(ex.map(ping, targets))

print("\n===== ARP 表(仅内网) =====")
arp = run("arp -a")
CAR_MAC = "88-a2-9e-29-e1-e0"
found = []
for line in arp.splitlines():
    if re.search(r"([0-9]{1,3}\.){3}[0-9]{1,3}\s+([0-9a-fA-F-]{17})", line):
        print("  " + line.strip())
        if CAR_MAC.lower() in line.lower().replace(":", "-"):
            found.append(line.strip())

print("\n===== 小车 MAC 命中 =====")
print("\n".join(found) if found else "  未找到 " + CAR_MAC)

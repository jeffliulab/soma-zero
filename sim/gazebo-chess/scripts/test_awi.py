"""W3 自测：对着跑起来的 gazebo-chess 世界服务(:8106)走一遍 AWI——
形状校验(perceive 只给 controllers/phase、不含棋盘真值；有相机图) + 真走一步(take_seat→start_game→move)。
全过 exit 0、失败 exit 1。前提：episode 仿真栈 + image_bridge + gazebo-chess 服务都在跑。
用法：python3 scripts/test_awi.py [base_url=http://localhost:8106] [from=e2] [to=e4]"""
import sys
import urllib.request
import json

BASE = sys.argv[1] if len(sys.argv) > 1 else "http://localhost:8106"
FRM = sys.argv[2] if len(sys.argv) > 2 else "e2"
TO = sys.argv[3] if len(sys.argv) > 3 else "e4"


def get(path):
    with urllib.request.urlopen(BASE + path, timeout=15) as r:
        return json.load(r)


def invoke(name, **args):
    data = json.dumps({"name": name, "args": args}).encode()
    req = urllib.request.Request(BASE + "/invoke", data=data, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=90) as r:
        return json.load(r)


def main() -> int:
    fails = []
    caps = get("/capabilities")
    names = [t["name"] for t in caps.get("tools", [])]
    if caps.get("name") != "gazebo-chess":
        fails.append(f"capabilities.name 应为 gazebo-chess, 实为 {caps.get('name')}")
    for need in ("take_seat", "start_game", "move", "resign"):
        if need not in names:
            fails.append(f"工具缺 {need}")
    per = get("/perceive")
    st = per.get("state", {})
    if set(st.keys()) != {"controllers", "phase"}:
        fails.append(f"perceive.state 应只含 controllers/phase, 实为 {list(st.keys())}")
    if any(k in json.dumps(per) for k in ("fen", "FEN", "board")):
        fails.append("perceive 疑似泄漏了棋盘真值")
    if not per.get("image_b64"):
        fails.append("perceive 没给相机图(image_b64)")
    print("capabilities tools:", names)
    print("perceive state:", st, "| image:", "有" if per.get("image_b64") else "无")

    print("take_seat:", invoke("take_seat", seat="white"))
    print("start_game:", invoke("start_game"))
    res = invoke("move", **{"from": FRM, "to": TO})
    print("move:", res)
    if not res.get("ok"):
        fails.append(f"move 失败：{res.get('message')}")

    if fails:
        print("FAIL:")
        for f in fails:
            print("  -", f)
        return 1
    print(f"PASS: AWI 形状对、{FRM}->{TO} 经 HTTP 驱动机械臂抓放成功")
    return 0


if __name__ == "__main__":
    sys.exit(main())

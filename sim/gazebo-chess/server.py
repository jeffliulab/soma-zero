"""gazebo-chess 世界服务：把 GazeboChessWorld 经 HTTP(AWI) 暴露成一个标准「世界」。

AWI(脑↔世界): GET /capabilities  GET /perceive  POST /invoke  GET /health
人类页/流(世界本地): GET /stream(MJPEG)  GET /

前提：episode 仿真栈在跑（headless 即可）+ image_bridge 在把相机图桥到 ROS。详见 README / 运行命令.md。
"""
from __future__ import annotations

import asyncio
import contextlib
import os

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, StreamingResponse

import config
from awi_mcp import build_awi_mcp
from world import GazeboChessWorld

_CORS = [o.strip() for o in os.getenv("ANIMA_CORS_ORIGINS", "http://localhost:3000").split(",") if o.strip()]
_HERE = os.path.dirname(os.path.abspath(__file__))

world = GazeboChessWorld()

# 世界说明书（= MCP prompt "guidance"；大脑读了就懂怎么跟我打交道）。
GAZEBO_GUIDANCE = (
    "我是「gazebo-chess」世界：Gazebo 里的物理国际象棋盘 + 一条真机械臂。你（大脑）负责决定走哪步棋，"
    "我负责物理执行。\n"
    "我给你三个物理原语：move（把一个子从 from 格夹到 to 格）、remove（把某格的子夹走拿出棋盘）、"
    "place（在某格摆上一个新子）。一手棋按棋规拆成原语按顺序调用：吃子=先 remove(被吃格) 再 move；"
    "过路兵=先 remove(被吃兵所在格，注意不是落点) 再 move；王车易位=王先车后两次 move（王走两格那步）；"
    "升变=move 兵到底线 → remove 兵 → place(该格, 升变子字母，大写白/小写黑)。\n"
    "对局模式下我内部有裁判：不合法的原语我会直接拒绝并告诉你为什么（机械臂不动）——"
    "被拒了就读原因、纠正想法，不要原样重试。我还有一个内置电脑对手：你每完成一手，它会立刻"
    "「瞬移」应一手（它不用机械臂）——我不会告诉你它走了什么，你下次感知时自己看画面认。\n"
    "感知（perceive）给你**多路相机画面**（默认两路：oblique 斜视、overhead 正俯视——"
    "state.cameras 按序标注每张图是哪路相机），除相机名单外 state 不含任何东西（棋盘真值绝不给你，你靠看）。\n"
    "棋盘边缘印有 a–h / 1–8 坐标：认格就按盘上印的坐标读，报格名前先对照坐标核一眼。\n"
    "一个原语要几十秒（真机械臂在动），失败了看我的报错决定怎么补救：报错会带自检分类"
    "（夹空=子还在原格，重试即可；放偏=报了子的实际落格，从那格把它 move 到本来要去的格；"
    "掉子=先感知找到它）。物理失败不影响棋规记录——把这手棋修完成，棋局才继续。\n"
    "子掉出棋盘找不回来时：用 place 在它**原来的格**补一枚同款子（我会当作「备用子恢复」，"
    "盘面与棋局记录重新对齐），然后重新走你该走的那手棋。"
)

# AWI（脑↔世界）走标准 MCP：世界作 MCP server 挂在 /mcp。
# 世界不声明任何服务：引擎顾问由大脑（Host）按 config.services() 自行挂载（标准 MCP 组装）。
mcp_asgi, mcp_lifespan = build_awi_mcp(world, guidance=GAZEBO_GUIDANCE, server_name="gazebo-chess")

# lifespan：包住 MCP 的 lifespan，进程退出时**完整关停**世界——删净相机模型（防「残留相机抢同一
# 话题 → 画面交替混流」，2026-07-02 实锤）+ 停 ROS spin/节点。uvicorn 重启/Ctrl+C 都走这里。
@contextlib.asynccontextmanager
async def _lifespan(app):
    async with mcp_lifespan(app):
        try:
            yield
        finally:
            await asyncio.to_thread(world.shutdown)


app = FastAPI(title="gazebo-chess world", lifespan=_lifespan)
app.add_middleware(CORSMiddleware, allow_origins=_CORS, allow_methods=["*"], allow_headers=["*"])
app.mount("/mcp", mcp_asgi)   # 大脑经此 list_tools / read_resource(感知) / call_tool / get_prompt(说明书)


# ===== AWI（脑↔世界）现在走标准 MCP（挂在 /mcp）；旧的 /capabilities /perceive /invoke 已撤 =====


@app.get("/health")
def health() -> dict:
    return {"ok": True, "arm_ready": world.ready}


@app.get("/status")  # 人类调试台·世界真值（上帝视角）：走世界本地，不进 AWI、绝不给 ANIMA
def status() -> dict:
    return world.debug_state()


@app.post("/reset")  # 人类侧「开新局」（网页按钮/curl）：走世界本地，不进 AWI——大脑不许重置现实
async def reset() -> dict:
    return await asyncio.to_thread(world.reset_board)


# ===== 人类页 / 流（世界本地，不进 AWI）=====
@app.get("/streams")  # 有哪几路相机直播（前端据此并列展示多画面；单相机世界没有此端点=回退单 /stream）
def streams() -> list[dict]:
    return [{"name": n, "url": f"/stream?cam={n}"} for n in config.cam_names()]


@app.get("/stream")   # 某一路相机的 MJPEG 直播；?cam=<名字>，缺省=第一路
async def stream(cam: str = "") -> StreamingResponse:
    async def gen():
        while True:
            jpg = await asyncio.to_thread(world.stream_jpeg, cam)
            if jpg is not None:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
            await asyncio.sleep(1 / config.STREAM_FPS)
    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")


@app.get("/")
def home() -> FileResponse:
    return FileResponse(os.path.join(_HERE, "web", "index.html"))

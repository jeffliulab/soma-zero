"""AWI-over-MCP 适配器（世界侧，自包含，仅依赖 mcp + anyio）。

把一个实现 `capabilities()/observe()/invoke()` 的世界对象，暴露成标准 **MCP server**：
  - **tools**    ← `world.capabilities()["tools"]`（含 JSON schema；kind∈{read,judge} → readOnlyHint=真）
  - **resource** ← `anima://observation`：读它返回 state(json text) + 画面(image/png blob)
  - **prompt**   ← `guidance`：世界说明书（世界作者写的一段自我介绍）

用法（世界的 FastAPI server.py）：
    from awi_mcp import build_awi_mcp
    mcp_asgi, mcp_lifespan = build_awi_mcp(world, guidance=GUIDANCE, server_name="camera")
    app = FastAPI(title="camera world", lifespan=mcp_lifespan)
    app.mount("/mcp", mcp_asgi)
    # 其余 /stream、/、控制端点照旧——它们是带外的人类页，不进 MCP。

**长时动作（v0.5 框架修正）**——物理世界的一个原语可能要几十秒（机械臂夹取搬运），为此：
  - 工具调用与感知一律下到工作线程（anyio.to_thread）跑：事件循环绝不被慢动作堵死，
    动作执行期间 /health、/stream、并发 MCP 请求照常响应（v0.4 的坑：同步跑在循环上，
    一次 move 冻住整个世界服务器，进度想发也发不出去）。
  - 进度采标 MCP `notifications/progress`：世界若想上报动作进度，只需在自己的 invoke 签名里
    声明 keyword-only 参数 `_progress`（`def invoke(self, name, *, _progress=None, **args)`），
    执行中调 `_progress(0.5, "已夹取，正在移向 e4")`。没声明的（即时动作的世界）零改动。
    带下划线是刻意的：防止和工具自己的参数撞名；构建时签名探测，不在运行时猜。

⚠️ 这份文件在每个世界目录各存一份副本（世界是独立进程/各自 venv，不共享 import）。改协议时四处同步
   （大脑仓 tests/test_awi_mcp_copies.py 字节级核对四份一致，漂移会挂测试）。
"""
from __future__ import annotations

import contextlib
import functools
import inspect
import json

import anyio
import anyio.to_thread
import mcp.types as t
from mcp.server.lowlevel import Server
from mcp.server.lowlevel.helper_types import ReadResourceContents
from mcp.server.streamable_http_manager import StreamableHTTPSessionManager

NON_MUTATING = {"read", "judge"}          # 非「改世界」的能力
OBSERVATION_URI = "anima://observation"   # 感知资源 URI（大脑侧 world_client.py 用同一串）
GUIDANCE_PROMPT = "guidance"              # 说明书提示词名


def build_awi_mcp(world=None, *, guidance: str = "",
                  server_name: str = "world", caps_fn=None, observe_fn=None, invoke_fn=None):
    """返回 (asgi_handler, lifespan)：挂到世界 FastAPI 的 /mcp，并用作 app 的 lifespan。

    默认从 `world` 取 capabilities()/observe()/invoke()；若世界的动作方法名不同（如 sim-desk 是 step），
    传 `invoke_fn=world.step` 覆盖即可（observe_fn/caps_fn 同理）。

    世界只暴露自己的 tools / observation / guidance——**不声明任何服务**：挂载服务（引擎顾问等）
    由大脑（Host）按自己的 config.services() 组装，server 之间互不相识（标准 MCP 模型）。
    """
    _caps = caps_fn or world.capabilities        # () -> {"tools":[...], ...}
    _observe = observe_fn or world.observe        # () -> (state, image_png|None)
    _invoke = invoke_fn or world.invoke           # (name, **args) -> {"ok","message","data"}
    # 签名探测：世界声明了 keyword-only `_progress` 才把进度上报函数传给它（即时世界零改动、不误传）。
    try:
        _wants_progress = "_progress" in inspect.signature(_invoke).parameters
    except (TypeError, ValueError):
        _wants_progress = False
    srv = Server(server_name)

    @srv.list_tools()
    async def _list_tools():
        caps = _caps()
        out = []
        for td in caps.get("tools", []):
            kind = td.get("kind", "tool")
            out.append(t.Tool(
                name=td["name"],
                description=td.get("description", ""),
                inputSchema=td.get("parameters") or {"type": "object", "properties": {}},
                annotations=t.ToolAnnotations(readOnlyHint=(kind in NON_MUTATING)),
            ))
        return out

    @srv.call_tool()
    async def _call_tool(name, arguments):
        kwargs = dict(arguments or {})
        if _wants_progress:
            ctx = srv.request_context
            token = ctx.meta.progressToken if ctx.meta else None
            session, req_id = ctx.session, ctx.request_id

            def _report(progress: float, message: str = "") -> None:
                """世界代码在工作线程里调的进度上报；客户端没带 progressToken 就静默跳过。"""
                if token is None:
                    return
                # related_request_id 必带：stateless 模式靠它把通知路由回本请求的 SSE 流。
                anyio.from_thread.run(functools.partial(
                    session.send_progress_notification, token, float(progress),
                    total=1.0, message=str(message) or None, related_request_id=req_id))

            kwargs["_progress"] = _report
        # 干活下工作线程：慢动作绝不堵事件循环（进度通知也才发得出去）。
        res = await anyio.to_thread.run_sync(functools.partial(_invoke, name, **kwargs))
        if not isinstance(res, dict):
            res = {"ok": True, "message": str(res)}
        ok = bool(res.get("ok", True))
        msg = res.get("message", "") or ("ok" if ok else "failed")
        data = res.get("data") or None
        # isError 精确表达成败（如非法着 = ok False → isError True）；data 走 structuredContent。
        return t.CallToolResult(
            content=[t.TextContent(type="text", text=msg)],
            structuredContent=data,
            isError=not ok,
        )

    @srv.list_resources()
    async def _list_resources():
        return [t.Resource(
            uri=OBSERVATION_URI, name="observation",
            description="当前画面 + 结构 state（大脑感知；绝不含世界真值）",
            mimeType="application/json",
        )]

    @srv.read_resource()
    async def _read_resource(uri):
        # 感知同样下工作线程（gazebo 的 observe 要拿世界锁 + spin ROS，不许堵循环）。
        state, image = await anyio.to_thread.run_sync(_observe)
        out = [ReadResourceContents(content=json.dumps(state or {}), mime_type="application/json")]
        # 画面两种形状（多相机是一等公民）：
        #   bytes                     单相机（老形状，向后兼容，单相机世界零改动）
        #   list[(name, bytes)]       多相机：依序回多个 png blob，名字顺序 = world 放进 state["cameras"] 的顺序
        #     （MCP blob 本身不带名字，顺序即对应关系——world 侧必须用同一个列表生成两者）
        # 没画面就不给 blob（大脑据此知道"暂时没画面"，绝不伪造一张图）。
        if isinstance(image, list):
            for _name, blob in image:
                if blob:
                    out.append(ReadResourceContents(content=blob, mime_type="image/png"))
        elif image:
            out.append(ReadResourceContents(content=image, mime_type="image/png"))
        return out

    @srv.list_prompts()
    async def _list_prompts():
        return [t.Prompt(name=GUIDANCE_PROMPT, description="世界说明书")] if guidance else []

    @srv.get_prompt()
    async def _get_prompt(name, arguments):
        return t.GetPromptResult(
            description="世界说明书",
            messages=[t.PromptMessage(role="user", content=t.TextContent(type="text", text=guidance))],
        )

    # json_response 必须为 False（SSE 应答模式）：JSON 模式下 SDK 只等 response、直接丢弃
    # notifications/progress（mcp 1.28.1 server/streamable_http.py），长动作的进度就传不到大脑。
    # 别为"简化"改回 True——那会静默弄断整条进度链。
    sm = StreamableHTTPSessionManager(app=srv, json_response=False, stateless=True)

    async def asgi(scope, receive, send):
        await sm.handle_request(scope, receive, send)

    @contextlib.asynccontextmanager
    async def lifespan(app):
        async with sm.run():
            yield

    return asgi, lifespan

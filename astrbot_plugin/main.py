"""VRChat for AstrBot: the bot's own VRChat desktop client as a platform.

The game runs on a Linux host next to ``bridge/vrc_bridge.py``; this plugin
registers the ``vrchat`` platform adapter (``vrchat_adapter.py``: realtime
voice in the room, quick actions for the voice model, text to the chatbox)
and the tools the room's chat uses to move the avatar, follow friends and go
places.

Needs the AstrBot Codex fork (``astrbot.core.voice``).
"""

from __future__ import annotations

import base64
import json
import time

import mcp.types

from astrbot.api import llm_tool
from astrbot.api.event import AstrMessageEvent, filter
from astrbot.api.star import Context, Star

from .vrchat_adapter import EMOTES, find_adapter


def _json(data) -> str:
    return json.dumps(data, ensure_ascii=False)


class VRChatPlugin(Star):
    """VRChat 平台：bot 自己的 VRChat 客户端，房间里实时语音对话，能移动、做表情、写 Chatbox、跟随白名单好友。"""

    def __init__(self, context: Context, config: dict | None = None) -> None:
        super().__init__(context)
        self.config = config or {}

    async def _act(self, event: AstrMessageEvent, action) -> str:
        """Runs ``action(adapter)`` for a tool of the VRChat chat; returns the
        tool result. Voice turns come as synthetic events of another platform
        name, so the chat is told by its platform id."""
        adapter = find_adapter(self.context)
        if adapter is None:
            return _json({"error": "VRChat 平台未运行"})
        if event.get_platform_id() != adapter.meta().id:
            return _json({"error": "只能在 VRChat 会话中使用"})
        try:
            return _json(await action(adapter))
        except Exception as exc:  # noqa: BLE001 - reported to the model
            return _json({"error": str(exc)})

    async def _picture(self, event: AstrMessageEvent, action) -> mcp.types.CallToolResult | str:
        """Like ``_act`` for a tool that shows the model a picture:
        ``action(adapter)`` returns (text, jpeg) or a text."""
        adapter = find_adapter(self.context)
        if adapter is None:
            return _json({"error": "VRChat 平台未运行"})
        if event.get_platform_id() != adapter.meta().id:
            return _json({"error": "只能在 VRChat 会话中使用"})
        try:
            result = await action(adapter)
        except Exception as exc:  # noqa: BLE001 - reported to the model
            return _json({"error": str(exc)})
        if isinstance(result, str):
            return result
        text, jpeg = result
        return mcp.types.CallToolResult(content=[
            mcp.types.TextContent(type="text", text=text),
            mcp.types.ImageContent(type="image", data=base64.b64encode(jpeg).decode(),
                                   mimeType="image/jpeg"),
        ])

    @llm_tool("vrchat_who")
    async def vrchat_who(self, event: AstrMessageEvent) -> str:
        """列出 VRChat 房间里现在的其他玩家（显示名）。
        """
        adapter = find_adapter(self.context)
        if adapter is not None and event.unified_msg_origin != adapter.room_umo:
            return _json({"error": "只能在 VRChat 语音线程配套的文字会话中使用"})

        async def who(adapter):
            return {"players": adapter.players()}

        return await self._act(event, who)

    @llm_tool("vrchat_view")
    async def vrchat_view(self, event: AstrMessageEvent, show: str = "around", name: str = "",
                          camera: str = "first"):
        """看 VRChat 里 bot 的画面。around（默认）：原地转一圈，前、右 / 后、左四向全景（2x2 拼图），可走位置统一编号，可用 vrchat_goto 前往；ahead：只看正前方（快）；map：边走边建的俯视地图（记过的地标、可站立平台 P1..）；last_seen：最后一次看到某位白名单好友时的画面，附时间和世界。

        Args:
            show(string): around、ahead、map 或 last_seen。
            name(string): last_seen 时的好友显示名；留空则取最近看到的那位。
            camera(string): around / ahead 时：first（默认，第一人称）或 third（临时切到第三人称从身后看，画面中间下方那个人就是自己，用来确认站在哪、看更大范围；看完自动切回第一人称，移动总是第一人称）。
        """
        # Only in the room's own chat: the text thread the room's voice
        # thread hands its tasks to (the persona lists this tool for every
        # chat).
        adapter = find_adapter(self.context)
        if adapter is not None and event.unified_msg_origin != adapter.room_umo:
            return _json({"error": "只能在 VRChat 语音线程配套的文字会话中使用"})
        if show == "map":

            async def mapped(adapter):
                jpeg, words = await adapter.map_view()
                return f"俯视地图（你在中间、朝上）。{words}", jpeg

            return await self._picture(event, mapped)
        if show == "ahead":

            async def ahead(adapter):
                jpeg = await adapter.view(camera)
                return f"正前方（{time.strftime('%H:%M:%S')}）。{adapter.nav_text}", jpeg

            return await self._picture(event, ahead)
        if show != "last_seen":

            async def around(adapter):
                jpeg = await adapter.look_around(camera)
                return f"四周（前、右 / 后、左）。{adapter.nav_text}", jpeg

            return await self._picture(event, around)

        async def last_seen(adapter):
            found = await adapter.last_seen(name)
            if found is None:
                return "还没有看到过白名单好友。"
            sighting, jpeg = found
            text = (f"最后一次看到 {sighting['name']}：{sighting['age_s']:.0f} 秒前，"
                    f"在 {sighting['world'] or '未知世界'}")
            return text, jpeg

        return await self._picture(event, last_seen)

    # -- admin commands, from any platform ---------------------------------

    @filter.command_group("vrc")
    def vrc(self):
        """VRChat 客户端管理（管理员）。"""

    @filter.permission_type(filter.PermissionType.ADMIN)
    @vrc.command("status")
    async def vrc_status(self, event: AstrMessageEvent):
        """VRChat 状态：桥接、客户端、游戏内账号、所在房间、Web API 登录、跟随。"""
        adapter = find_adapter(self.context)
        if adapter is None:
            yield event.plain_result("VRChat 平台未启用。")
            return
        lines = [f"桥接：{'已连接' if adapter.bridge_connected else '未连接'}（{adapter.base_url}）"]
        try:
            status = await adapter.request("GET", "/v1/status")
            social = await adapter.request("GET", "/v1/social")
        except Exception as exc:  # noqa: BLE001 - shown to the admin
            lines.append(f"无法读取状态：{exc}")
            yield event.plain_result("\n".join(lines))
            return
        me = status.get("self") or {}
        lines.append("客户端：" + (f"运行中，显存 {status.get('vram_mib')} MiB" if status.get("running")
                                  else "未运行"))
        if status.get("running"):
            lines.append(f"游戏内账号：{me.get('name') or '未登录/加载中'}")
            world = status.get("world") or "（不在世界中）"
            lines.append(f"所在：{world} {status.get('instance') or ''}".rstrip())
            players = [p["name"] for p in status.get("players", [])]
            lines.append(f"房间里：{', '.join(players) if players else '无其他玩家'}")
        account = social.get("me") or {}
        lines.append("Web API：" + (f"已登录 {account.get('name')}" if social.get("logged_in")
                                   else "未登录（需在服务器上运行 vrc_bridge.py login）"))
        follow = status.get("follow") or {}
        if follow.get("state", "idle") != "idle":
            lines.append(f"跟随：{follow.get('target')}（{follow.get('state')}）")
        yield event.plain_result("\n".join(lines))

    @filter.permission_type(filter.PermissionType.ADMIN)
    @vrc.command("start")
    async def vrc_start(self, event: AstrMessageEvent, url: str = ""):
        """启动 VRChat 客户端；可带 vrchat://launch 链接直接进房间（公开房间拒绝）。"""
        yield event.plain_result(await self._admin(lambda a: a.request(
            "POST", "/v1/game/start", {"url": url} if url else {}), "已发出启动指令，约半分钟进入游戏。"))

    @filter.permission_type(filter.PermissionType.ADMIN)
    @vrc.command("stop")
    async def vrc_stop(self, event: AstrMessageEvent):
        """关闭 VRChat 客户端。"""
        yield event.plain_result(await self._admin(
            lambda a: a.request("POST", "/v1/game/stop"), "已关闭 VRChat 客户端。"))

    @filter.permission_type(filter.PermissionType.ADMIN)
    @vrc.command("restart")
    async def vrc_restart(self, event: AstrMessageEvent):
        """重启 VRChat 客户端（回到默认的家）。"""

        async def restart(adapter):
            await adapter.request("POST", "/v1/game/stop")
            return await adapter.request("POST", "/v1/game/start", {})

        yield event.plain_result(await self._admin(restart, "已重启 VRChat 客户端，约半分钟进入游戏。"))

    async def _admin(self, action, done: str) -> str:
        adapter = find_adapter(self.context)
        if adapter is None:
            return "VRChat 平台未启用。"
        try:
            await action(adapter)
        except Exception as exc:  # noqa: BLE001 - shown to the admin
            return f"失败：{exc}"
        return done

    @llm_tool("vrchat_status")
    async def vrchat_status(self, event: AstrMessageEvent) -> str:
        """VRChat 当前状态：游戏是否在运行、所在世界和房间实例、房间里的其他玩家（显示名）。"""
        return await self._act(event, lambda a: a.request("GET", "/v1/status"))

    @llm_tool("vrchat_move")
    async def vrchat_move(
        self, event: AstrMessageEvent, direction: str, seconds: float = 1.0, run: bool = False
    ) -> str:
        """在 VRChat 里移动自己的身体（相对当前朝向），走完自动停下。

        Args:
            direction(string): forward、back、left、right 之一。
            seconds(number): 走多久，秒，最多 10；走几步约 1 秒。
            run(boolean): 是否奔跑。
        """
        return await self._act(event, lambda a: a.move(direction, seconds, run))

    @llm_tool("vrchat_turn")
    async def vrchat_turn(self, event: AstrMessageEvent, direction: str, seconds: float = 0.5) -> str:
        """在 VRChat 里原地左右转身。

        Args:
            direction(string): left 或 right。
            seconds(number): 转多久，秒，最多 5。
        """
        return await self._act(event, lambda a: a.turn(direction, seconds))

    @llm_tool("vrchat_look")
    async def vrchat_look(self, event: AstrMessageEvent, direction: str, amount: int = 200) -> str:
        """在 VRChat 里抬头或低头。

        Args:
            direction(string): up 或 down。
            amount(number): 幅度，100 为一点，400 为很多，最多 600。
        """
        return await self._act(event, lambda a: a.look(direction, amount))

    @llm_tool("vrchat_jump")
    async def vrchat_jump(self, event: AstrMessageEvent) -> str:
        """在 VRChat 里跳一下。"""
        return await self._act(event, lambda a: a.request("POST", "/v1/jump"))

    @llm_tool("vrchat_emote")
    async def vrchat_emote(self, event: AstrMessageEvent, name: str) -> str:
        """在 VRChat 里做一个表情动作（当前 avatar 支持默认表情菜单时）。

        Args:
            name(string): wave、clap、point、cheer、dance、backflip、sadness、die 之一。
        """
        if name not in EMOTES:
            return _json({"error": f"name 必须是 {', '.join(EMOTES)} 之一"})
        return await self._act(event, lambda a: a.request("POST", "/v1/emote", {"name": name}))

    @llm_tool("vrchat_stop")
    async def vrchat_stop(self, event: AstrMessageEvent) -> str:
        """立即停止 VRChat 里正在进行的移动和转身。"""
        return await self._act(event, lambda a: a.request("POST", "/v1/stop"))

    @llm_tool("vrchat_chatbox")
    async def vrchat_chatbox(self, event: AstrMessageEvent, text: str) -> str:
        """在 VRChat 头顶的 Chatbox 里显示文字（每条最多 144 字，长文自动分条）。

        Args:
            text(string): 要显示的文字，纯文本。
        """
        return await self._act(event, lambda a: a.request("POST", "/v1/chatbox", {"text": text}))

    @llm_tool("vrchat_social")
    async def vrchat_social(self, event: AstrMessageEvent) -> str:
        """VRChat 白名单好友的情况：各自在线与否、所在房间能否进入（公开房间不进）、是否开启跨房间跟随及正在跟随谁、Web API 是否已登录。"""

        async def social(adapter):
            status = await adapter.request("GET", "/v1/social")
            status["whitelist"] = [
                {key: entry.get(key) for key in ("name", "friend", "kind", "joinable")}
                for entry in status.get("whitelist", [])
            ]
            status.pop("me", None)
            return status

        return await self._act(event, social)

    @llm_tool("vrchat_follow_player")
    async def vrchat_follow_player(
        self, event: AstrMessageEvent, name: str = "", stop: bool = False, distance: float = 0.0
    ) -> str:
        """在当前 VRChat 房间里跟随某位玩家：始终面向对方并保持一定距离（看头顶名牌找人），直到叫停或对方离开房间。

        Args:
            name(string): 对方显示名；留空则跟白名单里优先级最高、且在本房间的好友。
            stop(boolean): true 表示停止跟随。
            distance(number): 与对方保持的距离，米（0.8 到 7）；不填或 0 用默认（约 1.5 米）。
        """
        return await self._act(event, lambda a: a.follow(name, stop, distance or None))

    @llm_tool("vrchat_follow_adjust")
    async def vrchat_follow_adjust(self, event: AstrMessageEvent, change: str) -> str:
        """调整正在进行的 VRChat 跟随：靠近一点、离远一点、原地别动（仍面向对方）、继续跟随。没在跟随时，stay 只是停下不动。

        Args:
            change(string): closer（靠近一点）、farther（离远一点）、stay（原地别动）、resume（继续跟）之一。
        """
        return await self._act(event, lambda a: a.follow_adjust(change))

    @llm_tool("vrchat_drive")
    async def vrchat_drive(self, event: AstrMessageEvent, steps: list,
                           view: str = "ahead") -> mcp.types.CallToolResult | str:
        """接管并驾驶 VRChat 角色做小动作（跳、侧移、按角度转身或抬头，行走最多 1 秒，全速 1 秒约 4 米；赶路或走一段距离用 vrchat_goto，如后退 2 米：bearing 180 distance 2），期间暂停自动跟随；完成后告诉你实际走了多远，默认返回正前方画面。接管前在跟随的，停止操作 10 秒后自动恢复跟随（或用 vrchat_autopilot 立即恢复）。

        Args:
            steps(array): 步骤列表，依次执行（合计最多 10 秒、24 步）。每步是同时按住的输入，持续 ms 毫秒：move（forward/back/left/right/forward-left/forward-right/back-left/back-right）、speed（0.2-1，1 约每秒 4 米，0.5 约 2 米）、run、jump（该步开始时跳）、turn（度，正为右）、look（度，正为上）、ms（0-3000）。例：[{"move":"forward","ms":600},{"move":"forward","jump":true,"ms":400},{"turn":-90}]
            view(string): 完成后返回的画面：ahead（默认，正前方）、around（四向全景）、none（不返回）。
        """

        async def drive(adapter):
            result, jpeg = await adapter.drive(steps, view)
            done = (f"完成：{result.get('steps')} 步，{result.get('ms', 0) / 1000:.1f} 秒。"
                    if result.get("ok") else "中途被停止。")
            moved = result.get("moved")
            if moved:
                done += (f"实际位移：向前 {moved['ahead_m']} 米、向右 {moved['right_m']} 米"
                         "（相对出发时的朝向，负数为后、左）。")
            return (done + f"当前画面：{adapter.nav_text}", jpeg) if jpeg else done

        return await self._picture(event, drive)

    @llm_tool("vrchat_goto")
    async def vrchat_goto(self, event: AstrMessageEvent, mark: int = -1, detour: str = "auto",
                          view: str = "ahead", climb: bool = False, bearing: float | None = None,
                          distance: float | None = None, cell: str = "",
                          landmark: str = "", platform: str = "") -> mcp.types.CallToolResult | str:
        """走到上一张画面里标号的可走位置（自动转向、行走，被挡住会自动绕行）；0 表示掉头。完成后默认返回正前方画面和新的可走位置。

        Args:
            mark(number): 上一张画面里的位置编号；0 掉头。
            detour(string): 绕行：auto（默认，自动选边）、left（从左边绕）、right（从右边绕）、none（不绕，挡住就停）。
            view(string): 完成后返回的画面：ahead（默认，正前方）、around（四向全景）、none（不返回）。
            climb(boolean): true 表示要站到那里的东西上面（桌子、坐墩、台阶）：径直走到跟前后向前跳上去，结果里的 jump.height_change_m 是落地后升高了多少米（约 0 即没上去）。
            bearing(number): 不用编号时，按正前方画面顶部刻度尺的角度前往（度，正为右，180 为身后），适合对准某个物体（如要跳上去的东西）或走一段距离。
            distance(number): 与 bearing 一起用：要走多少米（默认走到被挡住或没路为止）。
            cell(string): 不用编号时，正前方画面网格里目标所在的格子（底部字母 A-H、左侧数字 1-5，如 D4）：径直走到那东西跟前（配合 climb 跳上去）。
            landmark(string): 用 vrchat_note 记过的东西的名字：按地图规划路线前往（看不见也行）。
            platform(string): 地图上的可站立平台编号（P1..），配合 climb 跳上去。
        """

        async def goto(adapter):
            result, jpeg = await adapter.goto(int(mark), detour, view, bool(climb), bearing,
                                              distance, cell or None, landmark or None,
                                              platform or None)
            text = f"结果：{json.dumps(result, ensure_ascii=False)}。"
            return (text + adapter.nav_text, jpeg) if jpeg else text

        return await self._picture(event, goto)

    @llm_tool("vrchat_camera_y")
    async def vrchat_camera_y(self, event: AstrMessageEvent, action: str = "view",
                              horizon: float | None = None,
                              degrees: float | None = None) -> mcp.types.CallToolResult | str:
        """VRChat 视角的上下（Y 轴）。view：拍一张从抬头到低头的长图，带刻度（黄线 now 0 是当前朝向，上下每 10 度一条）；level：按长图里远处地平线所在的刻度转过去，并记为水平基准；set：按度数抬头（正）或低头（负）看东西。画面一直朝天或朝地时，先 view 再 level。

        Args:
            action(string): view、level 或 set。
            horizon(number): level 必填：长图里远处地平线所在刻度的度数（如 -30）。
            degrees(number): set 必填：抬头（正）或低头（负）多少度。
        """
        act = action if action in ("view", "level", "set") else "view"
        value = horizon if act == "level" else degrees
        if act != "view" and value is None:
            need = "horizon" if act == "level" else "degrees"
            return _json({"error": f"{act} 需要 {need}：对照 view 长图的刻度"})

        async def shown(adapter):
            jpeg = await adapter.camera_y(act, None if act == "view" else float(value))
            if act == "view":
                return ("从抬头到低头的长图，黄线 now 0 是当前朝向，每 10 度一条刻度。水平时远处地平线在 now 线上；"
                        "要水平就用 level，horizon 填地平线所在刻度。"), jpeg
            if act == "level":
                return "已转到地平线并记为水平基准；地平线应在黄线上，不在就再 view。", jpeg
            return "已调整，当前画面带刻度；下次看画面或走动会自动回到水平。", jpeg

        return await self._picture(event, shown)

    @llm_tool("vrchat_note")
    async def vrchat_note(self, event: AstrMessageEvent, name: str, cell: str) -> str:
        """把正前方画面里某个格子中的东西按名字记到地图上，之后可用 vrchat_goto landmark 找回（看不见也行）。

        Args:
            name(string): 简短名字，如 黄色坐墩。
            cell(string): 它在正前方画面网格里的格子，如 D4。
        """
        return await self._act(event, lambda a: a.note(name, cell))

    @llm_tool("vrchat_autopilot")
    async def vrchat_autopilot(self, event: AstrMessageEvent) -> str:
        """结束手动驾驶：如果之前在跟随某人，恢复自动跟随。"""
        return await self._act(event, lambda a: a.autopilot())

    @llm_tool("vrchat_follow_rooms")
    async def vrchat_follow_rooms(self, event: AstrMessageEvent, enabled: bool) -> str:
        """开启或关闭跨房间跟随：开启后 bot 会去白名单里优先级最高、且在可进入房间（非公开）的好友那里，对方换房间时跟过去（每次换房间要重启游戏，约半分钟）。

        Args:
            enabled(boolean): true 开启，false 关闭。
        """

        async def follow(adapter):
            adapter.social_config["follow"] = bool(enabled)
            return await adapter.request("POST", "/v1/social/config", {"follow": bool(enabled)})

        return await self._act(event, follow)

    @llm_tool("vrchat_join")
    async def vrchat_join(self, event: AstrMessageEvent, url: str) -> str:
        """重启 VRChat 进入某位白名单好友此刻所在的房间（其他房间一律拒绝）。约需半分钟，期间语音中断。

        Args:
            url(string): 该好友房间的 vrchat://launch?ref=vrchat.com&id=wrld_...:实例~... 链接。
        """

        async def join(adapter):
            # Anyone in the room may ask: only to where a whitelisted friend is
            # now (the bridge checks it, and the link, before it stops the
            # game). Elsewhere: an admin's /vrc start.
            return await adapter.request(
                "POST",
                "/v1/game/start",
                {"url": url, "restart": True, "whitelisted_only": True},
            )

        return await self._act(event, join)

"""The grant tracker's content: tracker/data/cards/*.json and the page files.

Run either way:
    python tests/test_tracker_content.py
    pytest tests/test_tracker_content.py -v

tracker/data/ is local-only (.gitignore), so on a fresh clone every content
test reports SKIP, counted apart from the passes and never as one. Point
TRACKER_DIR at another copy of the tracker folder (data/ and assets/) to check
that copy instead.

What is pinned, and why each one exists:
- card ids and order match the audit's _batches.json, file by file, and every
  card's status_before (or, without one, its status) equals the audit verdict
  in _audit_items.json: a status move always keeps its baseline;
- every status, area, priority and effort is a value the site can show;
- en / id / zh carry the same keys and list lengths, on cards and pages, and
  the zh text has no Simplified-only characters (the site is Traditional);
- every evidence line is dated ("updated YYYY-MM-DD: " or "audit 2026-10-05: "),
  the newest dated line comes first, and the card's `updated` field matches it;
- the per-area "Board, <date>: N DONE, M PARTIAL" counts on the overview and
  the final-gate notes equal the cards' own verdicts (stale counts read as
  current progress otherwise);
- a stated Brake -> Loiter count for the 8 Oct follow includes the count in
  that run's metrics.json (skipped when demo/out is not on disk);
- no "N/N stored runs resolve" while wp1_roundtrip.json sets runs aside that
  do not resolve;
- tools/check_claims.py finds no live withdrawn claim in the two page files;
- every rail and media key the content uses has a UI label in all three
  languages (tracker/assets/i18n.js) and a file behind it (app.js MEDIA);
- the 8 Oct ArduPilot-flown run is labelled with its own rail, and the
  Examples page carries the Gate G1 frame.

Each check that could pass vacuously has a self-test that feeds it the wrong
input and expects a refusal.
"""
import json
import os
import re
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TRACKER = Path(os.environ.get("TRACKER_DIR") or ROOT / "tracker")
DATA = TRACKER / "data"
ASSETS = TRACKER / "assets"
RUN_8OCT = ROOT / "demo" / "out" / "pas_ap_20261008_224440"
WP1_ROUNDTRIP = ROOT / "docs" / "data" / "wp1_roundtrip.json"
sys.path.insert(0, str(ROOT / "tools"))

SKIP = "SKIP"
LANGS = ("en", "id", "zh")
VERDICTS = {"DONE", "PARTIAL", "MISSING", "DRIFT"}
AREAS = {"WP1", "WP2", "WP3", "WP4", "ARCH", "CROSS", "CLAIMS"}
PRIOS = {"gate", "high", "medium", "low"}
EFFORTS = {"small", "medium", "large", "unknown"}
KPI_STATUS = {"measured", "partial", "not_measured", "wrong_definition"}
BUILT = {"yes", "partial", "no"}
EVIDENCE = re.compile(r"^(updated (\d{4}-\d{2}-\d{2}): |audit 2026-10-05: )")

# Characters that exist only in Simplified Chinese. tools/add_update.py holds
# the common slips; this adds the rest of the frequent ones and leaves out
# characters that are also normal Traditional usage (只, 台, 里, 后, 干, 范,
# 面, 系, 制).
EXTRA_SIMPLIFIED = set(
    "产业东车华场务刚单卫厂历压参双号听启员团围圆坏块坚备够夹奋奖宽寻导将尘尽属岁币师带帮并"
    "广庆库废异弃张弹归彻忆总恶惊惯戏战户执扩扫扬扰护担拟拥择换损摄敌断旧晓术杀杂权构枪栏树"
    "桥欢气汇汉泽洁济浏涂温满灭灯灵炼烦热爱牵状独猎环现电画畅疗盖监盘码确础离积称稳穷竞"
    "笔简类粮紧练组细终绍给络统继续维综绿编缩罗职联肃肤脑脚节获药虽补装视览觉计订讨训记讲"
    "许论访评诉词译诗话详语说请读课谁调谈谢负贡财责质购贵费资赛赶趋跃践转轮轻较辑输迁运远违"
    "连迟选递逻遗邮针钟钢钱铁银链销锁键门闭闻阅队阶际陆险随隐难顶项顺须预领频题额风馆驱骤么"
    "们这个为发说时实现过来对没还会进动问题关样经点种从开间无飞机区则检测图数据视频录试验证"
    "错误变显应该报设计约线条结认识让处网页优软长头标级档护层传载达态况适轨规齐乱亚仅众"
    "伞伟伤伦伪体价余佣侠侧侦俭债倾偿储儿兑党兰兴养兽内册农冯冻净凉减凑凤击凿刘"
    "创删别剂剑剧劝办劳势勋匀医协卖卢却厅厉厌县叙叠叹吓吗吨吴呐"
    "呜哑响哟唤啰啸喷嗳园圣坛坝坞坟坠垒垦垫埚堕墙壮声壳复夺奥妆"
    "妇妈姗娄娱婴孙学宁宝宠审宪宫宾寝寿尔尝尧屉届岂岗岛峡帅帐帘帧"
    "庐庙弥弯强录彦径忧怀怂怜恋恳恼悦悬惧惨惩惫惭惮愤愿"
    "扑抚抛抟抠抡抢拢拣拦拧拨挚挛挞挡挤挥挦捞捡捣据掳掴掷掸掺揽"
    "搀搁搂搅携摅摆摇摈摊撑撵撷擞攒敛数斋斓斩旷昙昼晋晒晕暂朴杨"
    "极枞枢枣枫柜栅栈栋桦梦检棂椭楼榄榉槛横樱欧歼残殴毁毕毙氢沟沪泞泪"
    "泸泻泼洒浅浆浇浊测浑浓涛涝涟涡涣润涧涨涩渊渐渔渗湾湿溃溅滚滤滥滨滩灾"
    "灿炉炜烁烂烛烟烧烨焕爷牍犹狈狭狮狱狲猪献獭玚玮珐珑琐琼瑶"
    "疖疟疠疡疮疯痉痪痫瘪瘫瘾盏盐睁瞒矫砖砚砺硕碍礼祷祸秃秆秽窃窍窑"
    "窜窝窥竖笃笋笼筑筛筹签箩纠纤级纪纫纬纯纱纲纳纵纷纸纹纺纽织绕"
    "绘绝绞绢绣绩绪绰绳绵绽缀缄缆缎缓缔缘缚缝缠缭缰缴罂罚罢聪肠"
    "肾肿胀胁胜脏脐脓脱脸腊腻腾舆舰舱艳艺芜苇苍苏茎荐荚荡荣莱莲莹萤营萧萨蓝蓦蔷"
    "蔼蕴虏虑虚蚀蚁蚂蛮蜕蝇蝉螀衅衬袄袜袭裤见观规觅触誉认讥让议讯讳"
    "讶讼讽设诀诈诊试诚诛诞询该诫诬诱诸诺谅谊谋谍谎"
    "谐谓谜谣谦谨谬谭谱贝贞贤败货贩贪贫贬贮贯贱贴贷贸贺贼贿赁赂赃赋赌赎赏赐"
    "赔赖赚赞赠踊踪蹿躯轧轩轰轴载轿辅辆辈辉辐辖辙辞辩辫边辽达过迈"
    "这逊邓邻郁郑酝酱酿释钉钓钙钝钞钠钥钦钧钩钮钳钻铃铅铜铝铲铸"
    "铺锄锅锈锋锐错锡锣锤锦锯锹锻镀镇镜镰长闪问闯闰闲闷闸闹阀阁阐阔阳阴阵"
    "陈陕隶雏雾霁静靥鞑韦韧韩页顷顽顾顿颁颂颇颈颓颖颗颜颠颤飘"
    "饥饭饮饰饱饲饵饶饼饿馈馒驮驯驰驳驴驶驹驻驼驾骂骄骆验骏骑骗骚髅鱼鲁鲜鸟鸡鸣鸭鸿鹅鹰黄"
    "齿龄龙龟")


def simplified_set():
    from add_update import SIMPLIFIED
    return set(SIMPLIFIED) | EXTRA_SIMPLIFIED


def _skip(why):
    print(f"      SKIP: {why}")
    pt = sys.modules.get("pytest")
    if pt is not None:
        pt.skip(why)
    return SKIP


def _load():
    """(cards by file, overview, flow) or None when the local data is absent."""
    files = sorted((DATA / "cards").glob("batch_*.json"))
    if not files:
        return None
    by_file = [(f.name, json.loads(f.read_text(encoding="utf-8"))) for f in files]
    ov = json.loads((DATA / "content_overview.json").read_text(encoding="utf-8"))
    fl = json.loads((DATA / "content_flow.json").read_text(encoding="utf-8"))
    return by_file, ov, fl


def _cards(by_file):
    return [c for _, cs in by_file for c in cs]


def _strings(o, path=()):
    if isinstance(o, dict):
        for k, v in o.items():
            yield from _strings(v, path + (k,))
    elif isinstance(o, list):
        for i, v in enumerate(o):
            yield from _strings(v, path + (i,))
    elif isinstance(o, str):
        yield path, o


def _trilingual_nodes(o, path=()):
    """Every dict that holds en / id / zh text: i18n blocks and results notes."""
    if isinstance(o, dict):
        for k, v in o.items():
            if k in ("i18n", "notes") and isinstance(v, dict) and set(v) & set(LANGS):
                yield path + (k,), v
            yield from _trilingual_nodes(v, path + (k,))
    elif isinstance(o, list):
        for i, v in enumerate(o):
            yield from _trilingual_nodes(v, path + (i,))


def _ui_blocks():
    """{lang: text of that language's block} from tracker/assets/i18n.js."""
    js = (ASSETS / "i18n.js").read_text(encoding="utf-8")
    starts = [(m.group(1), m.end()) for m in re.finditer(r"^\s{2}(en|id|zh): \{", js, re.M)]
    out = {}
    for n, (lang, s) in enumerate(starts):
        e = starts[n + 1][1] if n + 1 < len(starts) else len(js)
        out[lang] = js[s:e]
    return out


def _media_table():
    """{id: (kind, file)} from tracker/assets/app.js MEDIA."""
    js = (ASSETS / "app.js").read_text(encoding="utf-8")
    m = re.search(r"const MEDIA = \{(.*?)\n  \};", js, re.S)
    assert m, "app.js has no MEDIA table"
    return {k: (kind, f) for k, kind, f in re.findall(r'(\w+): \["(img|video)", "([^"]+)"\]', m.group(1))}


# ------------------------------------------------------------------ cards
def test_card_ids_and_order_match_the_audit_batches():
    d = _load()
    if d is None:
        return _skip("tracker/data/cards is not here (local-only data)")
    by_file, _, _ = d
    batches = json.loads((DATA / "_batches.json").read_text(encoding="utf-8"))
    assert len(batches) == len(by_file), (len(batches), [n for n, _ in by_file])
    for (name, cards), batch in zip(by_file, batches):
        assert [c["id"] for c in cards] == [b["id"] for b in batch], name
    audit = json.loads((DATA / "_audit_items.json").read_text(encoding="utf-8"))
    ids = [c["id"] for c in _cards(by_file)]
    assert len(ids) == len(set(ids)) == len(audit), (len(ids), len(set(ids)), len(audit))
    assert set(ids) == {a["id"] for a in audit}


def test_every_status_move_keeps_its_audit_baseline():
    d = _load()
    if d is None:
        return _skip("tracker/data/cards is not here (local-only data)")
    audit = {a["id"]: a["status"] for a in
             json.loads((DATA / "_audit_items.json").read_text(encoding="utf-8"))}
    bad = []
    for c in _cards(d[0]):
        base = c.get("status_before", c["status"])
        if base != audit[c["id"]]:
            bad.append(f"{c['id']}: baseline {base} vs audit {audit[c['id']]}")
    assert not bad, bad


def test_statuses_and_labels_are_values_the_site_can_show():
    d = _load()
    if d is None:
        return _skip("tracker/data/cards is not here (local-only data)")
    by_file, ov, fl = d
    bad = []
    for c in _cards(by_file):
        if c["status"] not in VERDICTS or c.get("status_before", "DONE") not in VERDICTS:
            bad.append(f"{c['id']} status")
        if c["area"] not in AREAS or c["priority"] not in PRIOS or c.get("effort", "unknown") not in EFFORTS:
            bad.append(f"{c['id']} area/priority/effort")
        if not isinstance(c["needs_pi"], bool):
            bad.append(f"{c['id']} needs_pi")
    bad += [f"kpi {k['key']}" for k in ov["kpis"] if k["status"] not in KPI_STATUS]
    bad += [f"timeline {t['i18n']['en']['name']}" for t in ov["timeline"] if t["status"] not in VERDICTS]
    bad += [f"gate item {i['i18n']['en']['name']}" for g in ov["gates"] for i in g["items"]
            if i["status"] not in VERDICTS]
    bad += [f"flow {n['key']}" for sec in ("grant_architecture", "grant_shield_steps")
            for n in fl[sec] if n.get("built", "no") not in BUILT]
    assert not bad, bad


def test_the_three_languages_carry_the_same_keys_and_list_lengths():
    d = _load()
    if d is None:
        return _skip("tracker/data/cards is not here (local-only data)")
    by_file, ov, fl = d
    bad = []
    nodes = [(c["id"], ("i18n",), c["i18n"]) for c in _cards(by_file)]
    nodes += [(name, p, n) for name, doc in (("overview", ov), ("flow", fl))
              for p, n in _trilingual_nodes(doc)]
    for who, path, tri in nodes:
        if set(tri) != set(LANGS):
            bad.append(f"{who} {path}: languages {sorted(tri)}")
            continue
        en, idn, zh = (tri[L] for L in LANGS)
        if isinstance(en, dict):
            if not (set(en) == set(idn) == set(zh)):
                bad.append(f"{who} {path}: keys differ")
                continue
            for k, v in en.items():
                if isinstance(v, list) and not (len(v) == len(idn[k]) == len(zh[k])):
                    bad.append(f"{who} {path}.{k}: {len(v)}/{len(idn[k])}/{len(zh[k])} items")
        elif not (type(en) is type(idn) is type(zh)):
            bad.append(f"{who} {path}: types differ")
    assert not bad, bad


def _simplified_hits(doc):
    simp = simplified_set()
    out = []
    for path, s in _strings(doc):
        if "zh" in path:
            hit = sorted({ch for ch in s if ch in simp})
            if hit:
                out.append(f"{list(path)}: {''.join(hit)}")
    return out


def test_the_check_for_simplified_characters_can_fail():
    assert _simplified_hits({"zh": "这个规则没有检测到飞机"}), "a Simplified sentence passed"
    assert not _simplified_hits({"zh": "這個規則沒有偵測到飛機，只在台北後面"}), "Traditional text refused"


def test_the_chinese_text_is_traditional():
    d = _load()
    if d is None:
        return _skip("tracker/data/cards is not here (local-only data)")
    by_file, ov, fl = d
    bad = []
    for c in _cards(by_file):
        bad += [f"{c['id']} {h}" for h in _simplified_hits({"i18n": {"zh": c["i18n"]["zh"]}})]
    bad += [f"overview {h}" for h in _simplified_hits(ov)]
    bad += [f"flow {h}" for h in _simplified_hits(fl)]
    assert not bad, bad[:20]


def test_every_evidence_line_is_dated_and_the_newest_comes_first():
    d = _load()
    if d is None:
        return _skip("tracker/data/cards is not here (local-only data)")
    bad = []
    for c in _cards(d[0]):
        dates = []
        for e in c["evidence"]:
            m = EVIDENCE.match(e)
            if not m:
                bad.append(f"{c['id']}: undated evidence: {e[:60]}")
            elif m.group(2):
                dates.append(m.group(2))
        if dates and dates[0] != max(dates):
            bad.append(f"{c['id']}: newest dated line is not first ({dates})")
        if (dates or "updated" in c) and c.get("updated") != (max(dates) if dates else None):
            bad.append(f"{c['id']}: updated {c.get('updated')} vs evidence {dates[:1]}")
    assert not bad, bad


# ------------------------------------------------------------------ pages
def _board_counts(text):
    """The verdict counts in a 'Board, <date>: ...' lead clause, any language."""
    head = re.split(r"[.。]", text, maxsplit=1)[0]
    return {v: int(n) for n, v in re.findall(r"(\d+) (DONE|PARTIAL|MISSING|DRIFT)", head)}


def test_the_board_counts_on_the_overview_match_the_cards():
    d = _load()
    if d is None:
        return _skip("tracker/data/cards is not here (local-only data)")
    by_file, ov, _ = d
    have = Counter((c["area"], c["status"]) for c in _cards(by_file))
    want = {a: {v: have[(a, v)] for v in VERDICTS if have[(a, v)]} for a in ("WP1", "WP2", "WP3", "WP4")}
    gate_items = ov["gates"][[g["key"] for g in ov["gates"]].index("final")]["items"][:4]
    texts = []
    for w in ov["wps"]:
        texts += [(w["key"], f"wps.{w['key']}.{L}", w["i18n"][L]["status"]) for L in LANGS]
    for area, item in zip(("WP1", "WP2", "WP3", "WP4"), gate_items):
        texts += [(area, f"gate {item['i18n']['en']['name']}.{L}", item["i18n"][L]["note"]) for L in LANGS]
    bad, seen = [], 0
    for area, where, text in texts:
        got = _board_counts(text)
        if not got:
            continue                                   # no count stated, nothing to check
        seen += 1
        if got != want[area]:
            bad.append(f"{where}: says {got}, cards say {want[area]}")
    assert seen, "no 'Board' count found at all: the check would pass on nothing"
    assert not bad, bad


def test_the_board_count_check_can_fail():
    assert _board_counts("Board, 9 Oct: 15 DONE, 10 PARTIAL. Open: x") == {"DONE": 15, "PARTIAL": 10}
    assert _board_counts("看板（10 月 9 日）：5 DONE、16 PARTIAL。待辦") == {"DONE": 5, "PARTIAL": 16}
    assert _board_counts("Board, 8 Oct: 16 DONE, 9 PARTIAL.") != {"DONE": 15, "PARTIAL": 10}


# Statements of a Brake -> Loiter count, in the three languages the site uses.
_BL = re.compile(r"(\d+)\s*(?:x\s*)?(?:Brake\s*->\s*Loiter|eskalasi Brake|次 Brake|次由 Brake)"
                 r"|Brake\s*->\s*Loiter\s+(seven|tujuh|eight|delapan|\d+)\s*(?:times|kali)"
                 r"|(七|八)次由 Brake")
_WORDS = {"seven": 7, "tujuh": 7, "七": 7, "eight": 8, "delapan": 8, "八": 8}


def _brake_loiter_problems(strings, n_run):
    """Strings that state Brake -> Loiter counts without the run's own count.
    A string may also give parts (5 over theta, 3 other) or a replay count
    beside the flown one; it must still state the flown total somewhere."""
    out = []
    for where, s in strings:
        found = []
        for m in _BL.finditer(s):
            g = next(x for x in m.groups() if x)
            found.append(int(g) if g.isdigit() else _WORDS[g])
        if found and n_run not in found and not re.search(
                rf"共 {n_run} 次|total {n_run}\b|{n_run} in all", s):
            out.append(f"{where}: {s[:100]}")
    return out


def test_the_brake_to_loiter_check_can_fail():
    assert _brake_loiter_problems([("x", "The FSM went Brake -> Loiter 7 times.")], 8)
    assert _brake_loiter_problems([("x", "共 7 次 Brake -> Loiter")], 8)
    assert not _brake_loiter_problems([("x", "8 Brake -> Loiter as flown, 2 Brake -> Loiter on the replay")], 8)


def test_a_stated_brake_to_loiter_count_is_the_runs_own():
    d = _load()
    if d is None:
        return _skip("tracker/data/cards is not here (local-only data)")
    metrics = RUN_8OCT / "metrics.json"
    if not metrics.is_file():
        return _skip("demo/out/pas_ap_20261008_224440 is not here (gitignored run data)")
    trans = json.loads(metrics.read_text(encoding="utf-8"))["fsm"]["transitions"]
    n_run = sum(1 for t in trans if t["from"] == "Brake" and t["to"] == "Loiter")
    assert n_run > 0, "the run's FSM record has no Brake -> Loiter: the check would be empty"
    by_file, ov, fl = d
    strings = [(c["id"], s) for c in _cards(by_file) for _, s in _strings(c)]
    strings += [("overview", s) for _, s in _strings(ov)] + [("flow", s) for _, s in _strings(fl)]
    bad = _brake_loiter_problems(strings, n_run)
    assert not bad, bad


def test_no_every_stored_run_resolves_while_some_are_set_aside():
    d = _load()
    if d is None:
        return _skip("tracker/data/cards is not here (local-only data)")
    sr = json.loads(WP1_ROUNDTRIP.read_text(encoding="utf-8"))["stored_runs"]
    aside = [r for r in sr.get("set_aside", []) if not r.get("resolves")]
    if not aside:
        return _skip("wp1_roundtrip.json sets no unresolved run aside: nothing to guard")
    by_file, ov, fl = d
    pats = [re.compile(rf"\b{n}/{n}\b") for n in {86, int(sr["n"])}]
    topic = re.compile(r"(?i)stored|resolv|census|tersimpan|已存|儲存")
    docs = [(c["id"], c) for c in _cards(by_file)] + [("overview", ov), ("flow", fl)]
    bad = [f"{who}: {s[:100]}" for who, doc in docs for _, s in _strings(doc)
           if topic.search(s) and any(p.search(s) for p in pats)]
    assert not bad, bad


def test_the_page_files_carry_no_withdrawn_claim():
    d = _load()
    if d is None:
        return _skip("tracker/data/cards is not here (local-only data)")
    import check_claims
    bad = []
    for name in ("content_overview.json", "content_flow.json"):
        text = (DATA / name).read_text(encoding="utf-8")
        bad += [f"{name}: {h.claim} line {h.line}" for h in check_claims.scan_text(text, f"tracker/data/{name}")]
    assert not bad, bad


# ------------------------------------------------------------------ assets
def test_every_rail_the_content_uses_has_a_label_in_three_languages():
    d = _load()
    if d is None:
        return _skip("tracker/data/cards is not here (local-only data)")
    _, _, fl = d
    rails = {x.get("rail", "airsim") for sec in ("examples", "results") for x in fl[sec]}
    blocks = _ui_blocks()
    assert set(blocks) == set(LANGS), sorted(blocks)
    bad = [f"rail_{r} missing in {L}" for r in sorted(rails) for L in LANGS
           if not re.search(rf'\brail_{re.escape(r)}: "[^"]+"', blocks[L])]
    assert not bad, bad


def test_every_media_key_resolves_to_a_file():
    d = _load()
    if d is None:
        return _skip("tracker/data/cards is not here (local-only data)")
    by_file, ov, fl = d
    table = _media_table()
    used = [("example", x["media"]) for x in fl["examples"] if x.get("media")]
    used += [(c["id"], m) for c in _cards(by_file) for m in c.get("media", [])]
    used += [(f"wps.{w['key']}", m) for w in ov["wps"] for m in w.get("media", [])]
    bad = []
    for who, mid in used:
        if "/" in mid:
            kind, f = ("video", mid.split("/", 1)[1]) if mid.startswith("video/") else ("img", mid)
        elif mid in table:
            kind, f = table[mid]
        else:
            bad.append(f"{who}: media {mid!r} not in app.js MEDIA")
            continue
        if not (ROOT / "docs" / kind / f).is_file():
            bad.append(f"{who}: {mid} -> docs/{kind}/{f} does not exist")
    assert not bad, bad


def test_the_ardupilot_flown_run_has_its_own_rail_and_g1_is_shown():
    d = _load()
    if d is None:
        return _skip("tracker/data/cards is not here (local-only data)")
    _, _, fl = d
    pas = [r for r in fl["results"] if r["run"].startswith("pas_ap_")]
    assert pas, "no Project AirSim + ArduPilot run in the results table"
    assert all(r.get("rail") == "pas_ap" for r in pas), [(r["run"], r.get("rail")) for r in pas]
    g1 = [x for x in fl["examples"] if x.get("media") == "g1_airborne"]
    assert len(g1) == 1, len(g1)
    assert g1[0]["date"] == "2026-10-08" and g1[0].get("rail") == "pas_ap", g1[0]
    for L in LANGS:
        assert all(g1[0]["i18n"][L].get(k) for k in ("title", "what", "proves", "not_proves")), L


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = skipped = 0
    for fn in fns:
        try:
            if fn() == SKIP:
                skipped += 1
                print(f"SKIP  {fn.__name__}")
                continue
            print(f"PASS  {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL  {fn.__name__}\n      {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}\n      {type(e).__name__}: {e}")
    tail = f", {skipped} skipped (not passed)" if skipped else ""
    print(f"\n{len(fns) - failed - skipped}/{len(fns)} passed{tail}")
    sys.exit(1 if failed else 0)

"""Render the ePlanSys tutorial video: keeping a secret with epistemic planning.

Every model drawn comes from traces.json (computed by del.py from the grounded
task); every planner line is from a real run. Frames are drawn with pycairo
and piped to ffmpeg as raw RGB.
"""
import json
import math
import subprocess
import sys

import cairo

W, H, FPS = 1280, 720, 30
OUT = sys.argv[1] if len(sys.argv) > 1 else 'tutorial.mp4'
TR = json.load(open('traces.json'))

# ── palette ──────────────────────────────────────────────────────────────────
BG = (0.984, 0.980, 0.969)
INK = (0.114, 0.141, 0.200)
MUTED = (0.42, 0.45, 0.50)
FAINT = (0.86, 0.86, 0.84)
PANEL = (1.0, 1.0, 1.0)
AG = {'scout': (0.184, 0.365, 0.620), 'relay': (0.227, 0.490, 0.267),
      'observer': (0.784, 0.392, 0.141)}
HAZ = (0.80, 0.62, 0.10)
OK = (0.20, 0.55, 0.30)
BAD = (0.75, 0.20, 0.18)
SANS, MONO, SERIF = 'Noto Sans', 'DejaVu Sans Mono', 'DejaVu Serif'


def ease(x):
    x = max(0.0, min(1.0, x))
    return x * x * (3 - 2 * x)


def lerp(a, b, t):
    return a + (b - a) * t


def mix(c1, c2, t):
    return tuple(lerp(a, b, t) for a, b in zip(c1, c2))


def rgba(cr, c, a=1.0):
    cr.set_source_rgba(c[0], c[1], c[2], a)


SYMBOLS = set('✓✗→◎☣✉—')


def _runs(s, font):
    runs, cur, curf = [], '', None
    for ch in s:
        f = 'DejaVu Sans' if (ch in SYMBOLS and font != MONO) else font
        if f != curf and cur:
            runs.append((cur, curf))
            cur = ''
        cur += ch
        curf = f
    if cur:
        runs.append((cur, curf))
    return runs


def _face(cr, f, size, bold, italic):
    cr.select_font_face(f, cairo.FONT_SLANT_ITALIC if italic else cairo.FONT_SLANT_NORMAL,
                        cairo.FONT_WEIGHT_BOLD if bold else cairo.FONT_WEIGHT_NORMAL)
    cr.set_font_size(size)


def text(cr, s, x, y, size, color=INK, font=SANS, bold=False, italic=False,
         align='left', alpha=1.0):
    runs = _runs(s, font)
    width = 0.0
    for r, f in runs:
        _face(cr, f, size, bold, italic)
        width += cr.text_extents(r).x_advance
    if align == 'center':
        x -= width / 2
    elif align == 'right':
        x -= width
    rgba(cr, color, alpha)
    for r, f in runs:
        _face(cr, f, size, bold, italic)
        cr.move_to(x, y)
        cr.show_text(r)
        x += cr.text_extents(r).x_advance
    return width


def wrap(cr, s, x, y, size, width, color=INK, alpha=1.0, lh=1.35, font=SANS):
    cr.select_font_face(font, cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_NORMAL)
    cr.set_font_size(size)
    line, yy = '', y
    for word in s.split():
        trial = (line + ' ' + word).strip()
        if cr.text_extents(trial).x_advance > width and line:
            text(cr, line, x, yy, size, color, font=font, alpha=alpha)
            yy += size * lh
            line = word
        else:
            line = trial
    if line:
        text(cr, line, x, yy, size, color, font=font, alpha=alpha)
    return yy


def kw(cr, x, y, agent, size=22, neg=False, alpha=1.0, color=INK):
    """Draw  [¬] Kw_agent contaminated  in math style."""
    if neg:
        x += text(cr, '¬', x, y, size, color, font=SERIF, alpha=alpha) + 2
    x += text(cr, 'Kw', x, y, size, color, font=SERIF, italic=True, alpha=alpha)
    x += text(cr, agent, x + 1, y + size * 0.28, size * 0.62, AG[agent], font=SANS,
              bold=True, alpha=alpha) + 6
    text(cr, 'contaminated', x, y, size, color, font=SERIF, italic=True, alpha=alpha)


def rounded(cr, x, y, w, h, r):
    cr.new_sub_path()
    cr.arc(x + w - r, y + r, r, -math.pi / 2, 0)
    cr.arc(x + w - r, y + h - r, r, 0, math.pi / 2)
    cr.arc(x + r, y + h - r, r, math.pi / 2, math.pi)
    cr.arc(x + r, y + r, r, math.pi, 3 * math.pi / 2)
    cr.close_path()


def panel(cr, x, y, w, h, title=None, alpha=1.0):
    rounded(cr, x, y, w, h, 10)
    rgba(cr, PANEL, alpha)
    cr.fill_preserve()
    rgba(cr, FAINT, alpha)
    cr.set_line_width(1.2)
    cr.stroke()
    if title:
        text(cr, title.upper(), x + 18, y + 28, 12, MUTED, bold=True, alpha=alpha)


# ── robots and map ───────────────────────────────────────────────────────────
DEPOT = (150, 380)
SITE = (480, 250)
HOME = {'scout': (115, 340), 'relay': (175, 420), 'observer': (300, 160)}


def rover(cr, x, y, agent, alpha=1.0, scale=1.0):
    c = AG[agent]
    s = 18 * scale
    rgba(cr, c, alpha)
    rounded(cr, x - s, y - s * 0.7, 2 * s, 1.4 * s, 5)
    cr.fill()
    rgba(cr, INK, alpha)
    for dx in (-s * 0.65, s * 0.65):
        cr.arc(x + dx, y + s * 0.75, s * 0.28, 0, 2 * math.pi)
        cr.fill()
    rgba(cr, PANEL, alpha)
    cr.arc(x + s * 0.35, y - s * 0.1, s * 0.22, 0, 2 * math.pi)
    cr.fill()
    text(cr, agent, x, y + s + 24, 13, c, bold=True, align='center', alpha=alpha)


def drone(cr, x, y, alpha=1.0, t=0.0):
    c = AG['observer']
    rgba(cr, c, alpha)
    cr.set_line_width(3)
    for a in (math.pi / 4, 3 * math.pi / 4):
        cr.move_to(x - 20 * math.cos(a), y - 20 * math.sin(a))
        cr.line_to(x + 20 * math.cos(a), y + 20 * math.sin(a))
        cr.stroke()
    for a in (math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4):
        px, py = x + 20 * math.cos(a), y + 20 * math.sin(a)
        rgba(cr, c, alpha * 0.25)
        cr.arc(px, py, 9, 0, 2 * math.pi)
        cr.fill()
        rgba(cr, c, alpha)
        r = 9 * abs(math.cos(t * 30 + a))
        cr.move_to(px - r, py)
        cr.line_to(px + r, py)
        cr.stroke()
    cr.arc(x, y, 7, 0, 2 * math.pi)
    cr.fill()
    text(cr, 'observer', x, y + 44, 13, c, bold=True, align='center', alpha=alpha)


def world_map(cr, pos, alpha=1.0, t=0.0, site_reveal=0.0, extras=None):
    panel(cr, 40, 80, 590, 470, 'World', alpha)
    # ground grid
    rgba(cr, FAINT, alpha * 0.6)
    cr.set_line_width(1)
    for gx in range(70, 620, 40):
        cr.move_to(gx, 100)
        cr.line_to(gx, 535)
    for gy in range(110, 540, 40):
        cr.move_to(55, gy)
        cr.line_to(615, gy)
    cr.stroke()
    # road
    rgba(cr, (0.80, 0.80, 0.78), alpha)
    cr.set_line_width(16)
    cr.move_to(*DEPOT)
    cr.curve_to(300, 380, 360, 250, SITE[0], SITE[1])
    cr.stroke()
    # depot
    rgba(cr, (0.93, 0.93, 0.91), alpha)
    rounded(cr, 70, 300, 160, 160, 8)
    cr.fill()
    text(cr, 'depot', 150, 485, 13, MUTED, align='center', alpha=alpha)
    # site
    sx, sy = SITE
    rgba(cr, HAZ, alpha * 0.18)
    cr.arc(sx, sy, 62, 0, 2 * math.pi)
    cr.fill()
    rgba(cr, HAZ, alpha)
    cr.set_line_width(2)
    cr.set_dash([6, 5])
    cr.arc(sx, sy, 62, 0, 2 * math.pi)
    cr.stroke()
    cr.set_dash([])
    q = 1 - site_reveal
    text(cr, '?', sx, sy + 16, 46, mix(HAZ, BAD, site_reveal), bold=True, align='center',
         alpha=alpha * q)
    if site_reveal > 0:
        ra = alpha * site_reveal
        rgba(cr, BAD, ra)
        cr.move_to(sx, sy - 30)
        cr.line_to(sx + 32, sy + 24)
        cr.line_to(sx - 32, sy + 24)
        cr.close_path()
        cr.fill()
        text(cr, '!', sx, sy + 17, 34, PANEL, bold=True, align='center', alpha=ra)
        text(cr, 'contaminated', sx, sy - 74, 13, BAD, bold=True, align='center', alpha=ra)
    text(cr, 'site', sx, sy + 90, 13, MUTED, align='center', alpha=alpha)
    if extras:
        cr.save()
        rounded(cr, 41, 81, 588, 468, 10)
        cr.clip()
        extras(cr)
        cr.restore()
    for a in ('scout', 'relay'):
        rover(cr, *pos[a], a, alpha)
    drone(cr, *pos['observer'], alpha, t)


# ── Kripke models ────────────────────────────────────────────────────────────
def short(w):
    """Display name of a world from its product-update history."""
    parts = w.split('.')
    if parts[-1] == 'e-team-dirty':
        return 'told'
    if parts[-1] == 'e-open-dirty':
        return 'told'
    return ''


def world_node(cr, x, y, contaminated, designated, alpha=1.0, tag=''):
    r = 34
    rgba(cr, PANEL, alpha)
    cr.arc(x, y, r, 0, 2 * math.pi)
    cr.fill()
    rgba(cr, BAD if contaminated else OK, alpha)
    cr.set_line_width(2.5)
    cr.arc(x, y, r, 0, 2 * math.pi)
    cr.stroke()
    if designated > 0:
        rgba(cr, INK, alpha * designated)
        cr.set_line_width(2)
        cr.arc(x, y, r + 7, 0, 2 * math.pi)
        cr.stroke()
    lbl = 'contam.' if contaminated else 'clean'
    text(cr, lbl, x, y + 5, 14, BAD if contaminated else OK, bold=True, align='center',
         alpha=alpha)
    if tag:
        text(cr, tag, x, y + 22, 11, MUTED, align='center', alpha=alpha)


def agent_edge(cr, p, q, agent, k, n, alpha=1.0, directed=False):
    (x1, y1), (x2, y2) = p, q
    dx, dy = x2 - x1, y2 - y1
    L = math.hypot(dx, dy) or 1
    ux, uy = dx / L, dy / L
    ox, oy = -uy * (k - (n - 1) / 2) * 7, ux * (k - (n - 1) / 2) * 7
    a = (x1 + ux * 44 + ox, y1 + uy * 44 + oy)
    b = (x2 - ux * 44 + ox, y2 - uy * 44 + oy)
    rgba(cr, AG[agent], alpha)
    cr.set_line_width(2.6)
    cr.move_to(*a)
    cr.line_to(*b)
    cr.stroke()
    if directed:
        ang = math.atan2(uy, ux)
        cr.move_to(*b)
        cr.line_to(b[0] - 11 * math.cos(ang - 0.4), b[1] - 11 * math.sin(ang - 0.4))
        cr.line_to(b[0] - 11 * math.cos(ang + 0.4), b[1] - 11 * math.sin(ang + 0.4))
        cr.close_path()
        cr.fill()


def draw_model(cr, M, pos, alpha=1.0, wf=None, ef=None, desig=None):
    """Draw model M with world positions `pos`. wf/ef/desig give per-world alpha."""
    wf = wf or {}
    ef = ef or (lambda a, b, i: 1.0)
    pairs = {}
    for i in ('scout', 'relay', 'observer'):
        for w, succ in M['rel'][i].items():
            for v in succ:
                if v == w:
                    continue
                key = tuple(sorted((w, v)))
                sym = w in M['rel'][i][v]
                pairs.setdefault(key, {})[i] = (sym, w, v)
    for (w, v), ags in pairs.items():
        names = [i for i in ('scout', 'relay', 'observer') if i in ags]
        for k, i in enumerate(names):
            sym, src, dst = ags[i]
            al = alpha * min(wf.get(w, 1), wf.get(v, 1)) * ef(w, v, i)
            agent_edge(cr, pos[src], pos[dst], i, k, len(names), al, directed=not sym)
    for w in M['worlds']:
        d = (desig or {}).get(w, 1.0 if w in M['designated'] else 0.0)
        world_node(cr, *pos[w], 'contaminated' in M['labels'][w], d,
                   alpha * wf.get(w, 1), short(w))


def model_panel(cr, alpha, title='Epistemic state (Kripke model)', stats=None):
    panel(cr, 650, 80, 590, 340, title, alpha)
    x = 668
    for i in ('scout', 'relay', 'observer'):
        rgba(cr, AG[i], alpha)
        cr.set_line_width(3)
        cr.move_to(x, 404)
        cr.line_to(x + 22, 404)
        cr.stroke()
        x += 28 + text(cr, i, x + 28, 409, 12, MUTED, alpha=alpha) + 18
    tw = text(cr, 'designated', 1222, 409, 12, MUTED, align='right', alpha=alpha)
    cx = 1222 - tw - 14
    rgba(cr, MUTED, alpha)
    cr.set_line_width(1.2)
    cr.new_path()
    cr.arc(cx, 405, 4, 0, 2 * math.pi)
    cr.stroke()
    cr.new_path()
    cr.arc(cx, 405, 7, 0, 2 * math.pi)
    cr.stroke()
    if stats:
        text(cr, stats, 1222, 108, 13, MUTED, align='right', font=MONO, alpha=alpha)


def goal_panel(cr, status, alpha=1.0, flash=None):
    panel(cr, 650, 432, 590, 118, 'Goal', alpha)
    rows = [('scout', False), ('relay', False), ('observer', True)]
    for k, ((ag, neg), st) in enumerate(zip(rows, status)):
        y = 482 + k * 27
        x = 690
        if st is None:
            rgba(cr, MUTED, alpha)
            cr.set_line_width(1.5)
            cr.arc(x, y - 6, 8, 0, 2 * math.pi)
            cr.stroke()
        else:
            c = OK if st else BAD
            f = 1.0
            if flash is not None and flash[0] == k:
                f = 1 + 0.5 * math.sin(math.pi * min(1, flash[1]))
            rgba(cr, c, alpha)
            cr.arc(x, y - 6, 9 * f, 0, 2 * math.pi)
            cr.fill()
            text(cr, '✓' if st else '✗', x, y - 1, 13, PANEL, bold=True, align='center',
                 alpha=alpha)
        kw(cr, x + 22, y, ag, 19, neg, alpha)


def header(cr, step, title, alpha=1.0):
    text(cr, step, 40, 50, 14, MUTED, bold=True, alpha=alpha)
    text(cr, title, 40 + 14 + len(step) * 8.6, 50, 20, INK, bold=True, alpha=alpha)


def caption(cr, s, alpha=1.0):
    rgba(cr, FAINT, alpha)
    cr.set_line_width(1)
    cr.move_to(40, 588)
    cr.line_to(1240, 588)
    cr.stroke()
    wrap(cr, s, 40, 624, 20, 1200, INK, alpha)


def brand(cr, alpha=1.0):
    text(cr, 'ePlanSys', 1240, 50, 16, INK, bold=True, align='right', alpha=alpha)


# ── world positions for each model ───────────────────────────────────────────
def positions(M):
    pos = {}
    for w in M['worlds']:
        c = 'contaminated' in M['labels'][w]
        if short(w) == 'told':
            pos[w] = (1095, 175)
        elif c:
            pos[w] = (1095, 315)
        else:
            pos[w] = (805, 315)
    if len(M['worlds']) == 1:
        pos[M['worlds'][0]] = (945, 245)
    return pos


DIRTY = TR['policy-dirty']
BCAST = TR['broadcast']


# ── scenes ───────────────────────────────────────────────────────────────────
def s_title(cr, t, d):
    a = ease(t / 0.8) * (1 - ease((t - d + 0.6) / 0.6))
    text(cr, 'ePlanSys', W / 2, 250, 30, MUTED, bold=True, align='center', alpha=a)
    text(cr, 'Keeping a secret with epistemic planning', W / 2, 320, 42, INK, bold=True,
         align='center', alpha=a)
    text(cr, 'A worked example: planning over what robots know, and executing it on ROS 2',
         W / 2, 370, 20, MUTED, align='center', alpha=a)
    rgba(cr, FAINT, a)
    cr.set_line_width(1)
    cr.move_to(W / 2 - 220, 410)
    cr.line_to(W / 2 + 220, 410)
    cr.stroke()
    text(cr, 'Site survey mission  ·  plank + Aletheia + PlanSys2', W / 2, 445, 16, MUTED,
         align='center', alpha=a)


def s_mission(cr, t, d):
    a = ease(t / 0.8) * (1 - ease((t - d + 0.5) / 0.5))
    header(cr, '1', 'The mission', a)
    brand(cr, a)
    world_map(cr, HOME, a, t)
    panel(cr, 650, 80, 590, 470, 'Specification (EPDDL)', a)
    rows = [
        ('Agents', 'scout, relay (the team); observer (not controlled by the team)'),
        ('Unknown', 'whether the site is contaminated'),
        ('Actions', 'goto-site, scan, broadcast (public), relay (private message)'),
    ]
    y = 140
    for k, (h, v) in enumerate(rows):
        ra = a * ease((t - 0.6 - 0.5 * k) / 0.6)
        text(cr, h, 672, y, 15, MUTED, bold=True, alpha=ra)
        y = wrap(cr, v, 772, y, 16, 440, INK, ra) + 36
    ga = a * ease((t - 2.4) / 0.8)
    text(cr, 'Goal', 672, 330, 15, MUTED, bold=True, alpha=ga)
    for k, (ag, neg) in enumerate([('scout', False), ('relay', False), ('observer', True)]):
        kw(cr, 772, 332 + k * 40, ag, 24, neg, a * ease((t - 2.6 - 0.5 * k) / 0.6))
    na = a * ease((t - 4.4) / 0.8)
    wrap(cr, 'The third conjunct requires that an agent stays ignorant. '
         'It is not a fact about the world, so PDDL cannot express it.',
         672, 480, 15, 540, MUTED, na)
    caption(cr, 'Three robots and a site that may be contaminated. The team must find out; '
            'the observer must not.', a * ease((t - 0.5) / 0.8))


def s_model(cr, t, d):
    a = ease(t / 0.6) * (1 - ease((t - d + 0.5) / 0.5))
    header(cr, '2', 'The initial epistemic state', a)
    brand(cr, a)
    world_map(cr, HOME, a, t)
    M = DIRTY[0]['model']
    model_panel(cr, a, stats='2 worlds · 2 designated')
    p = positions(M)
    draw_model(cr, M, p, a * ease((t - 0.4) / 0.8),
               ef=lambda w, v, i: ease((t - 1.2 - 0.4 * ['scout', 'relay', 'observer'].index(i)) / 0.5))
    goal_panel(cr, DIRTY[0]['goal'], a * ease((t - 2.5) / 0.6))
    caption(cr, 'Two possible worlds, both designated. Each agent has an edge between them: '
            'no robot can tell whether the site is contaminated.', a * ease((t - 0.8) / 0.8))


PLANNER_LINES = [
    '$ epistemic_planner --task survey-team.json --strategy aostar',
    '[parser] Loaded: 4 atoms, 3 agents, 2 worlds (2 designated), 30 actions',
    '[aostar] Solution found at depth 3  Expanded=20  Generated=26',
    '[validator] OK — 2 leaves, 4 branches checked',
]


def s_plan(cr, t, d):
    a = ease(t / 0.6) * (1 - ease((t - d + 0.5) / 0.5))
    header(cr, '3', 'Planning: AND/OR search over Kripke models', a)
    brand(cr, a)
    rounded(cr, 40, 80, 1200, 170, 10)
    rgba(cr, (0.11, 0.13, 0.17), a)
    cr.fill()
    for k, line in enumerate(PLANNER_LINES):
        start = 0.5 + 0.9 * k
        n = int(len(line) * ease((t - start) / 0.7))
        col = (0.75, 0.85, 0.95) if k == 0 else (0.85, 0.87, 0.90)
        if k == 3:
            col = (0.55, 0.85, 0.60)
        text(cr, line[:n], 66, 125 + k * 32, 16, col, font=MONO, alpha=a)
    # policy tree
    ta = a * ease((t - 4.2) / 0.8)
    panel(cr, 40, 265, 1200, 285, 'Returned policy', ta)
    nodes = [
        ((260, 405), 'goto-site scout', None),
        ((560, 405), 'scan scout', 'sensing'),
        ((900, 335), 'relay-dirty scout → relay', None),
        ((900, 475), 'relay-clean scout → relay', None),
    ]
    edges = [(0, 1, ''), (1, 2, 'dirty'), (1, 3, 'clean')]
    wid = [220 if 'relay' not in n[1] else 250 for n in nodes]
    for i, j, lbl in edges:
        (x1, y1), (x2, y2) = nodes[i][0], nodes[j][0]
        rgba(cr, MUTED, ta)
        cr.set_line_width(1.8)
        cr.move_to(x1 + wid[i] / 2, y1)
        cr.line_to(x2 - wid[j] / 2, y2)
        cr.stroke()
        if lbl:
            text(cr, lbl, (x1 + x2) / 2 + 10, (y1 + y2) / 2 - 8, 13, MUTED, italic=True,
                 align='center', alpha=ta)
    for (x, y), lbl, kind in nodes:
        w = 250 if 'relay' in lbl else 220
        rounded(cr, x - w / 2, y - 22, w, 44, 8)
        rgba(cr, (0.93, 0.95, 0.99) if kind else PANEL, ta)
        cr.fill_preserve()
        rgba(cr, MUTED, ta)
        cr.set_line_width(1.3)
        cr.stroke()
        text(cr, lbl, x, y + 6, 15, INK, bold=True, align='center', alpha=ta)
    text(cr, 'goal holds', 1130, 340, 13, OK, bold=True, align='center', alpha=ta)
    text(cr, 'goal holds', 1130, 480, 13, OK, bold=True, align='center', alpha=ta)
    caption(cr, 'The planner searches the space of epistemic states. The policy branches on the '
            'scan, and both branches tell relay over the private channel.',
            a * ease((t - 4.6) / 0.8))


def exec_frame(cr, t, a, step, title, cap, pos, M_from, M_to, k, goal, site_rev,
               extras=None, morph=None, flash=None, stats=None, show_brand=True):
    header(cr, step, title, a)
    if show_brand:
        brand(cr, a)
    world_map(cr, pos, a, t, site_rev, extras)
    model_panel(cr, a, stats=stats)
    if morph:
        morph(cr, a)
    else:
        draw_model(cr, M_to, positions(M_to), a)
    goal_panel(cr, goal, a, flash)
    caption(cr, cap, a * ease((t - 0.3) / 0.8))


def along_road(u):
    # cubic bezier depot -> site, matching world_map's road
    p0, p1, p2, p3 = DEPOT, (300, 380), (360, 250), SITE
    x = (1 - u) ** 3 * p0[0] + 3 * (1 - u) ** 2 * u * p1[0] + 3 * (1 - u) * u ** 2 * p2[0] + u ** 3 * p3[0]
    y = (1 - u) ** 3 * p0[1] + 3 * (1 - u) ** 2 * u * p1[1] + 3 * (1 - u) * u ** 2 * p2[1] + u ** 3 * p3[1]
    return x, y


ON_SITE = (SITE[0] - 92, SITE[1] + 48)


def s_goto(cr, t, d):
    a = ease(t / 0.5) * (1 - ease((t - d + 0.4) / 0.4))
    u = ease((t - 0.8) / 3.0)
    sx, sy = along_road(u)
    sx, sy = lerp(HOME['scout'][0], sx, ease(t / 0.8)), lerp(HOME['scout'][1], sy, ease(t / 0.8))
    if u >= 1:
        sx, sy = lerp(SITE[0], ON_SITE[0], ease((t - 3.8) / 0.6)), lerp(SITE[1], ON_SITE[1], ease((t - 3.8) / 0.6))
    pos = dict(HOME, scout=(sx, sy))
    M = DIRTY[1]['model']
    exec_frame(cr, t, a, '4', 'goto-site scout  ·  public, ontic',
               'Everyone sees the scout drive to the site. The world changes (the scout is on site), '
               'but no agent learns anything about contamination: the model keeps both worlds.',
               pos, None, M, 1, DIRTY[1]['goal'], 0.0, stats='2 worlds · 2 designated')


def scan_extras(t):
    def f(cr):
        for k in range(3):
            ph = (t * 1.2 + k / 3) % 1
            rgba(cr, AG['scout'], 0.5 * (1 - ph))
            cr.set_line_width(2)
            cr.arc(ON_SITE[0], ON_SITE[1], 20 + 50 * ph, -1.2, 0.2)
            cr.stroke()
    return f


def s_scan(cr, t, d):
    a = ease(t / 0.5) * (1 - ease((t - d + 0.4) / 0.4))
    pos = dict(HOME, scout=ON_SITE)
    M0, M1 = DIRTY[1]['model'], DIRTY[2]['model']
    u = ease((t - 1.6) / 1.2)
    p0, p1 = positions(M0), positions(M1)

    def morph(cr, al):
        # same two worlds; scout's edge fades, clean world loses its designation
        clean0 = [w for w in M0['worlds'] if 'contaminated' not in M0['labels'][w]][0]
        dirty0 = [w for w in M0['worlds'] if w != clean0][0]
        draw_model(cr, M0, p0, al,
                   ef=lambda w, v, i: (1 - u) if i == 'scout' else 1.0,
                   desig={clean0: 1 - u, dirty0: 1.0})
    goal = DIRTY[1]['goal'] if t < 3.0 else DIRTY[2]['goal']
    exec_frame(cr, t, a, '5', 'scan scout  ·  semi-private sensing',
               'The scout observes the result. relay and the observer see that a scan happened but not '
               'its result: only the scout\'s edge disappears.',
               pos, M0, M1, 2, goal, ease((t - 1.0) / 0.8),
               extras=scan_extras(t) if t < 3.2 else None, morph=morph,
               flash=(0, (t - 3.0) / 0.5) if t >= 3.0 else None,
               stats='2 worlds · 1 designated' if t >= 2.4 else '2 worlds · 2 designated')


def s_relay(cr, t, d):
    a = ease(t / 0.5) * (1 - ease((t - d + 0.4) / 0.4))
    pos = dict(HOME, scout=ON_SITE)
    M0, M1 = DIRTY[2]['model'], DIRTY[3]['model']
    u = ease((t - 2.0) / 1.6)

    def msg(cr):
        ph = ease((t - 0.6) / 1.4)
        if 0 < ph < 1 or (t > 0.6 and t < 2.4):
            x = lerp(ON_SITE[0], HOME['relay'][0], ph)
            y = lerp(ON_SITE[1], HOME['relay'][1], ph)
            rgba(cr, AG['relay'], 0.35)
            cr.set_line_width(2)
            cr.set_dash([4, 5])
            cr.move_to(*ON_SITE)
            cr.line_to(*HOME['relay'])
            cr.stroke()
            cr.set_dash([])
            rgba(cr, AG['relay'])
            rounded(cr, x - 14, y - 10, 28, 20, 4)
            cr.fill()
            text(cr, '✉', x, y + 6, 15, PANEL, align='center')
        if t > 0.8:
            text(cr, 'oblivious', HOME['observer'][0] + 48, HOME['observer'][1] + 5, 13,
                 AG['observer'], italic=True, alpha=ease((t - 0.8) / 0.6))

    def morph(cr, al):
        p1 = positions(M1)
        told = [w for w in M1['worlds'] if short(w) == 'told'][0]
        dirty_nil = [w for w in M1['worlds'] if 'contaminated' in M1['labels'][w] and w != told][0]
        start = dict(p1)
        start[told] = p1[dirty_nil]
        cur = {w: (lerp(start[w][0], p1[w][0], u), lerp(start[w][1], p1[w][1], u)) for w in p1}
        draw_model(cr, M1, cur, al, wf={told: 0.25 + 0.75 * u},
                   desig={w: (1.0 if w == told else 0.0) for w in M1['worlds']})
    if t < 2.0:
        def morph0(cr, al):
            draw_model(cr, M0, positions(M0), al)
        mm = morph0
        goal = DIRTY[2]['goal']
    else:
        mm = morph
        goal = DIRTY[3]['goal']
    exec_frame(cr, t, a, '6', 'relay-dirty scout → relay  ·  private announcement',
               'scout tells relay over a private link. The observer does not notice, so the model gains a '
               'world: the "told" world is actual, and the observer still considers the old two possible.',
               pos, M0, M1, 3, goal, 1.0, extras=msg, morph=mm,
               flash=(1, (t - 3.6) / 0.5) if t >= 3.6 else None,
               stats='3 worlds · 1 designated' if t >= 2.0 else '2 worlds · 1 designated')
    if t > 4.6:
        ba = a * ease((t - 4.6) / 0.6)
        rounded(cr, 1040, 470, 182, 56, 8)
        rgba(cr, OK, ba * 0.12)
        cr.fill()
        text(cr, 'goal holds', 1131, 493, 15, OK, bold=True, align='center', alpha=ba)
        text(cr, 'mission complete', 1131, 514, 13, OK, align='center', alpha=ba)


def s_broadcast(cr, t, d):
    a = ease(t / 0.5) * (1 - ease((t - d + 0.5) / 0.5))
    pos = dict(HOME, scout=ON_SITE)
    M0, M1 = BCAST[2]['model'], BCAST[3]['model']
    u = ease((t - 2.0) / 1.2)

    def waves(cr):
        for k in range(4):
            ph = ((t - 0.6) * 0.8 + k / 4) % 1
            if t < 0.6:
                continue
            rgba(cr, BAD, 0.45 * (1 - ph))
            cr.set_line_width(2)
            cr.arc(ON_SITE[0], ON_SITE[1], 20 + 330 * ph, 0, 2 * math.pi)
            cr.stroke()

    def morph(cr, al):
        p0 = positions(M0)
        clean = [w for w in M0['worlds'] if 'contaminated' not in M0['labels'][w]][0]
        dirty = [w for w in M0['worlds'] if w != clean][0]
        target = (945, 245)
        cur = dict(p0)
        cur[dirty] = (lerp(p0[dirty][0], target[0], u), lerp(p0[dirty][1], target[1], u))
        draw_model(cr, M0, cur, al, wf={clean: 1 - u},
                   ef=lambda w, v, i: 1 - u)
    goal = BCAST[2]['goal'] if t < 2.6 else BCAST[3]['goal']
    exec_frame(cr, t, a, '7', 'Counterfactual: broadcast-dirty scout  ·  public announcement',
               'A broadcast reaches every agent, the observer included. The model collapses to one world '
               'and the third conjunct fails. This is why the planner never chooses it.',
               pos, M0, M1, 3, goal, 1.0, extras=waves, morph=morph,
               flash=(2, (t - 2.6) / 0.5) if t >= 2.6 else None,
               stats='1 world · 1 designated' if t >= 2.6 else '2 worlds · 1 designated',
               show_brand=False)
    rgba(cr, BAD, a * 0.9)
    rounded(cr, 1100, 30, 140, 28, 6)
    cr.fill()
    text(cr, 'NOT IN THE POLICY', 1170, 49, 11, PANEL, bold=True, align='center', alpha=a)
    if t > 3.2:
        ba = a * ease((t - 3.2) / 0.6)
        rounded(cr, 1040, 470, 182, 56, 8)
        rgba(cr, BAD, ba * 0.12)
        cr.fill()
        text(cr, 'goal violated', 1131, 493, 15, BAD, bold=True, align='center', alpha=ba)
        text(cr, 'observer knows', 1131, 514, 13, BAD, align='center', alpha=ba)


def s_close(cr, t, d):
    a = ease(t / 0.8)
    text(cr, 'The channel is chosen by its audience,', W / 2, 220, 34, INK, bold=True,
         align='center', alpha=a)
    text(cr, 'and the audience follows from the goal.', W / 2, 266, 34, INK, bold=True,
         align='center', alpha=a)
    b = ease((t - 1.2) / 0.8)
    rounded(cr, W / 2 - 330, 330, 660, 120, 10)
    rgba(cr, (0.11, 0.13, 0.17), b)
    cr.fill()
    text(cr, '$ sudo apt install ros-humble-eplansys-demo', W / 2 - 300, 372, 17,
         (0.85, 0.87, 0.90), font=MONO, alpha=b)
    text(cr, '$ ros2 launch eplansys_demo survey_launch.py', W / 2 - 300, 412, 17,
         (0.55, 0.85, 0.60), font=MONO, alpha=b)
    c = ease((t - 2.2) / 0.8)
    text(cr, 'eplansys.github.io/eplansys', W / 2, 510, 20, AG['scout'], bold=True,
         align='center', alpha=c)
    text(cr, 'github.com/ePlanSys/eplansys', W / 2, 542, 16, MUTED, align='center', alpha=c)


SCENES = [
    (s_title, 4.0), (s_mission, 8.5), (s_model, 7.5), (s_plan, 9.0),
    (s_goto, 6.5), (s_scan, 7.5), (s_relay, 9.0), (s_broadcast, 8.0), (s_close, 6.0),
]


def main():
    total = sum(d for _, d in SCENES)
    n = int(total * FPS)
    surf = cairo.ImageSurface(cairo.FORMAT_RGB24, W, H)
    ff = subprocess.Popen(
        ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr0',
         '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-',
         '-c:v', 'libx264', '-preset', 'slow', '-crf', '24', '-pix_fmt', 'yuv420p',
         '-movflags', '+faststart', OUT],
        stdin=subprocess.PIPE)
    poster_t = 4.0 + 8.5 + 7.5 + 9.0 + 6.5 + 7.5 + 7.0
    for f in range(n):
        tg = f / FPS
        cr = cairo.Context(surf)
        cr.set_antialias(cairo.ANTIALIAS_BEST)
        rgba(cr, BG)
        cr.paint()
        acc = 0.0
        for fn, d in SCENES:
            if tg < acc + d:
                cr.save()
                if fn not in (s_title, s_close):
                    cr.translate(0, 8)
                fn(cr, tg - acc, d)
                cr.restore()
                break
            acc += d
        # progress bar
        rgba(cr, FAINT)
        cr.rectangle(0, H - 4, W, 4)
        cr.fill()
        rgba(cr, AG['scout'])
        cr.rectangle(0, H - 4, W * tg / total, 4)
        cr.fill()
        surf.flush()
        ff.stdin.write(bytes(surf.get_data()))
        if f == int(poster_t * FPS):
            surf.write_to_png(OUT.rsplit('.', 1)[0] + '-poster.png')
    ff.stdin.close()
    ff.wait()
    print(f'{n} frames, {total:.1f} s -> {OUT}')


if __name__ == '__main__':
    main()

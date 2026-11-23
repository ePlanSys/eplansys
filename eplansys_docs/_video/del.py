"""Minimal DEL product update over a plank-grounded task, to drive the video.

Computes the multi-pointed Kripke model after each action of a policy branch,
contracted to its bisimulation quotient, and evaluates the goal conjuncts.
"""
import json
import sys


def holds(f, M, w):
    if isinstance(f, dict) and 'formula' in f and len(f) == 1:
        return holds(f['formula'], M, w)
    if f == 'true':
        return True
    if f == 'false':
        return False
    if isinstance(f, str):
        return f in M['labels'][w]
    if 'connective' in f:
        c = f['connective']
        if c == 'not':
            return not holds(f['formula'], M, w)
        if c == 'and':
            return all(holds(g, M, w) for g in f['formulas'])
        if c == 'or':
            return any(holds(g, M, w) for g in f['formulas'])
        raise ValueError(c)
    m = f['modality-name']
    (i,) = f['modality-index']
    succ = M['rel'][i][w]
    g = f['formula']
    box = all(holds(g, M, v) for v in succ)
    if m == 'box':
        return box
    if m == 'diamond':
        return any(holds(g, M, v) for v in succ)
    kw = all(holds(g, M, v) for v in succ) or all(not holds(g, M, v) for v in succ)
    if m == 'Kw.box':
        return kw
    if m == 'Kw.diamond':
        return not kw
    raise ValueError(m)


def model_from_task(task):
    s = task['initial-state']
    return {
        'worlds': list(s['worlds']),
        'rel': {i: {w: list(v) for w, v in r.items()} for i, r in s['relations'].items()},
        'labels': {w: set(s['labels'][w]) for w in s['worlds']},
        'designated': set(s['designated']),
    }


def obs_type(action, agent, M, w):
    for t, cond in action['observability-conditions'][agent].items():
        if holds(cond, M, w):
            return t
    raise ValueError('no observability for ' + agent)


def update(M, action, observed=None):
    """Product update. `observed` restricts the designated events (sensing)."""
    E = action['events']
    pre = action['preconditions']
    W = [(w, e) for w in M['worlds'] for e in E if holds(pre[e], M, w)]
    name = {p: f'{p[0]}.{p[1]}' for p in W}
    labels = {}
    for (w, e) in W:
        lab = set(M['labels'][w])
        eff = (action.get('effects') or {}).get(e) or {}
        for atom, cond in eff.items():
            if holds(cond, M, w):
                lab.add(atom)
            else:
                lab.discard(atom)
        labels[name[(w, e)]] = lab
    rel = {}
    for i in M['rel']:
        rel[i] = {}
        for (w, e) in W:
            t = obs_type(action, i, M, w)
            Q = action['relations'][t]
            rel[i][name[(w, e)]] = [name[(v, f)] for (v, f) in W
                                   if v in M['rel'][i][w] and f in Q[e]]
    des_e = set(action['designated']) if observed is None else {observed}
    des = {name[(w, e)] for (w, e) in W if w in M['designated'] and e in des_e}
    return contract({'worlds': [name[p] for p in W], 'rel': rel,
                     'labels': labels, 'designated': des})


def contract(M):
    # restrict to worlds reachable from the designated set
    seen, todo = set(M['designated']), list(M['designated'])
    while todo:
        w = todo.pop()
        for i in M['rel']:
            for v in M['rel'][i][w]:
                if v not in seen:
                    seen.add(v)
                    todo.append(v)
    W = [w for w in M['worlds'] if w in seen]
    # partition refinement
    block = {w: frozenset(M['labels'][w]) for w in W}
    while True:
        sig = {w: (block[w], tuple(sorted(
            (i, frozenset(block[v] for v in M['rel'][i][w])) for i in M['rel'])),
            w in M['designated']) for w in W}
        ids = {}
        new = {w: ids.setdefault(sig[w], len(ids)) for w in W}
        if len(set(new.values())) == len(set(block.values())):
            break
        block = new
    rep = {}
    for w in W:
        rep.setdefault(new[w], w)
    keep = sorted(rep.values(), key=W.index)
    to = {w: rep[new[w]] for w in W}
    return {
        'worlds': keep,
        'rel': {i: {w: sorted({to[v] for v in M['rel'][i][w]}, key=keep.index)
                    for w in keep} for i in M['rel']},
        'labels': {w: M['labels'][w] for w in keep},
        'designated': {to[w] for w in M['designated']},
    }


def goal_status(task, M):
    out = []
    for g in task['goal']['formula']['formulas']:
        out.append(all(holds(g, M, w) for w in M['designated']))
    return out


def dump(M):
    return {
        'worlds': M['worlds'],
        'labels': {w: sorted(M['labels'][w]) for w in M['worlds']},
        'designated': sorted(M['designated']),
        'rel': M['rel'],
    }


if __name__ == '__main__':
    task = json.load(open(sys.argv[1]))
    A = task['actions']
    runs = {
        'policy-dirty': [('goto-site_scout', None), ('scan_scout', 'e-scan-dirty'),
                         ('relay-dirty_scout_relay', None)],
        'policy-clean': [('goto-site_scout', None), ('scan_scout', 'e-scan-clean'),
                         ('relay-clean_scout_relay', None)],
        'broadcast': [('goto-site_scout', None), ('scan_scout', 'e-scan-dirty'),
                      ('broadcast-dirty_scout', None)],
    }
    result = {}
    for k, steps in runs.items():
        M = model_from_task(task)
        trace = [{'action': None, 'model': dump(M), 'goal': goal_status(task, M)}]
        for a, obs in steps:
            M = update(M, A[a], obs)
            trace.append({'action': a, 'observed': obs, 'model': dump(M),
                          'goal': goal_status(task, M)})
        result[k] = trace
        print(k)
        for t in trace:
            m = t['model']
            print(f"  {str(t['action']):28} {len(m['worlds'])} worlds, "
                  f"{len(m['designated'])} designated  goal={t['goal']}")
    json.dump(result, open(sys.argv[2], 'w'), indent=1)

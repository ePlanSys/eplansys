# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
``ros2 plansys2 epistemic`` verb.

The terminal talks to the problem expert: ``get problem``, ``set predicate``,
``set goal``. This is the same conversation with the epistemic state, which is
what holds the things a predicate cannot say — that an agent knows something,
or knows whether it is so.

It lives here rather than in ``plansys2_terminal`` because that package is
built by every distribution's workflow, and reaching the epistemic services
from C++ would make it depend on the epistemic message package to do so. A
Python verb imports them at run time and costs the classical build nothing.
"""

import json
import sys

import rclpy

from ros2cli.node.strategy import add_arguments, NodeStrategy
from ros2cli.verb import VerbExtension


def _call(node, srv_type, name, request, timeout=5.0):
    """Call one service, or explain why it could not be called."""
    client = node.create_client(srv_type, name)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            print(
                f'{name} is not available. Is the epistemic state running?\n'
                'Start it with epistemic_state:=True on either bringup launch '
                'file.',
                file=sys.stderr,
            )
            return None

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            print(f'{name} did not answer within {timeout:g}s', file=sys.stderr)
            return None
        return future.result()
    finally:
        node.destroy_client(client)


def _report(response):
    """Print a service error, and say whether the call succeeded."""
    if response is None:
        return False
    if not response.success:
        print(response.error, file=sys.stderr)
        return False
    return True


class EpistemicVerb(VerbExtension):
    """Inspect and change what the agents know."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument(
            'command', nargs='?', default='show',
            choices=['show', 'check', 'goal', 'announce', 'apply',
                     'domain', 'action'],
            help=(
                'show: the model and the goal; '
                'check <formula>: does it hold now; '
                'goal [<formula>]: report the goal, or set it; '
                'announce <formula>: everyone just learned this; '
                'apply <action>: advance the model by an executed action; '
                'domain: what the EPDDL domain declares; '
                'action <name>: one action\'s event model and who observes it'
            ),
        )
        parser.add_argument(
            'argument', nargs='?', default='',
            help='The formula or action the command takes.',
        )
        parser.add_argument(
            '--observed', default='',
            help=(
                'apply: the outcome that was observed, named by its event. '
                'Needed only when the model designates several worlds and so '
                'cannot determine the outcome itself.'
            ),
        )
        parser.add_argument(
            '--timeout', type=float, default=5.0,
            help='Seconds to wait for the epistemic state (default: 5)',
        )

    def main(self, *, args):
        # Imported here rather than at module scope: ros2cli loads every verb
        # to build its help, and a missing epistemic message package would
        # then break `ros2 plansys2` as a whole rather than this one verb.
        from plansys2_epistemic_msgs.srv import (
            Announce, ApplyAction, CheckFormula, GetEpistemicActionDetails,
            GetEpistemicDomain, GetGoal, SetGoal,
        )

        handlers = {
            'show': lambda node: self._show(node, args),
            'check': lambda node: self._check(node, args, CheckFormula),
            'goal': lambda node: self._goal(node, args, GetGoal, SetGoal),
            'announce': lambda node: self._announce(node, args, Announce),
            'apply': lambda node: self._apply(node, args, ApplyAction),
            'domain': lambda node: self._domain(node, args, GetEpistemicDomain),
            'action': lambda node: self._action(node, args, GetEpistemicActionDetails),
        }

        # The real rclpy node, not the strategy wrapper. NodeStrategy forwards
        # unknown attributes to a DirectNode, but rclpy's spin functions take a
        # Node and set attributes on it, which would land on the wrapper
        # instead of on the node the executor is spinning.
        with NodeStrategy(args) as strategy:
            return handlers[args.command](strategy.direct_node.node)

    def _domain(self, node, args, GetEpistemicDomain):
        """Print what the EPDDL domain declares.

        The classical terminal's `get domain actions` reads the PDDL domain and
        is blind to all of this: an epistemic action is an event model with
        per-agent observability, and none of that has a PDDL surface.
        """
        response = _call(
            node, GetEpistemicDomain, 'epistemic_domain/get_domain',
            GetEpistemicDomain.Request(), args.timeout,
        )
        if response is None:
            return 1
        if not response.success:
            print(response.error, file=sys.stderr)
            return 1

        print(f'frame: {"KD45 (belief)" if response.kd45 else "S5 (knowledge)"}')
        print(f'partially observable: {"yes" if response.partial_obs else "no"}')
        print(f'agents: {" ".join(response.agents)}')
        print(f'atoms:  {" ".join(response.atoms)}')
        print('actions:')
        for name, sensing in zip(response.actions, response.sensing):
            print(f'  {name}{"  [sensing]" if sensing else ""}')
        return 0

    def _action(self, node, args, GetEpistemicActionDetails):
        """Print one action's event model and who observes what of it."""
        if not args.argument:
            print('action takes the name of an action', file=sys.stderr)
            return 1

        request = GetEpistemicActionDetails.Request()
        request.action = args.argument

        response = _call(
            node, GetEpistemicActionDetails,
            'epistemic_domain/get_action_details', request, args.timeout,
        )
        if response is None:
            return 1
        if not response.success:
            print(response.error, file=sys.stderr)
            return 1

        designated = set(response.designated_events)
        print(f'{args.argument}: {len(response.events)} events, '
              f'{len(designated)} of them possible')
        for name, pre, eff in zip(
                response.events, response.preconditions, response.effects):
            mark = '*' if name in designated else ' '
            print(f'  {mark} {name}')
            if pre:
                print(f'      when: {pre}')
            if eff:
                print(f'      does: {eff}')
        print('observability:')
        for line in response.observability:
            print(f'  {line}')
        return 0

    def _show(self, node, args):
        """Print the latched state: the shape of the model and the goal."""
        from std_msgs.msg import String

        received = []
        sub = node.create_subscription(
            String, 'epistemic_state/state', received.append,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            ),
        )
        try:
            end = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
            while not received and node.get_clock().now().nanoseconds < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            node.destroy_subscription(sub)

        if not received:
            print(
                'The epistemic state published nothing. It is either not '
                'running or not yet active.',
                file=sys.stderr,
            )
            return 1

        try:
            state = json.loads(received[-1].data)
        except json.JSONDecodeError:
            print(received[-1].data)
            return 0

        print('Epistemic state:\n')
        print(f"  agents      {state.get('agents', '?')}")
        print(f"  atoms       {state.get('atoms', '?')}")
        # Worlds are the possibilities the agents entertain; the designated
        # ones are those that could be the actual world. One designated world
        # means the state knows which world it is in.
        print(f"  worlds      {state.get('worlds', '?')}")
        print(f"  designated  {state.get('designated', '?')}")

        goal = state.get('goal', '')
        if goal:
            source = 'from the task' if state.get('goal_from_task') else 'set'
            holds = state.get('goal_holds')
            status = 'holds' if holds else 'does not hold'
            print(f'\n  goal        {goal}  ({source}, {status})')
        else:
            print('\n  goal        none')
        return 0

    def _check(self, node, args, CheckFormula):
        if not args.argument:
            print('check needs a formula, e.g. "(Kw c1 muddy_c1)"', file=sys.stderr)
            return 1

        request = CheckFormula.Request()
        request.formula = args.argument
        response = _call(
            node, CheckFormula, 'epistemic_state/check_formula', request, args.timeout)
        if not _report(response):
            return 1

        print('holds' if response.holds else 'does not hold')
        return 0

    def _goal(self, node, args, GetGoal, SetGoal):
        if args.argument:
            request = SetGoal.Request()
            request.goal = args.argument
            response = _call(
                node, SetGoal, 'epistemic_state/set_goal', request, args.timeout)
            if not _report(response):
                return 1
            print(
                f'goal set: {args.argument}'
                f"  ({'already holds' if response.holds else 'does not hold yet'})")
            return 0

        response = _call(
            node, GetGoal, 'epistemic_state/get_goal', GetGoal.Request(), args.timeout)
        if not _report(response):
            return 1

        if not response.goal:
            print('no goal')
            return 0

        source = 'from the task' if response.from_task else 'set'
        status = 'holds' if response.holds else 'does not hold'
        print(f'{response.goal}  ({source}, {status})')
        return 0

    def _announce(self, node, args, Announce):
        if not args.argument:
            print('announce needs a formula, e.g. "(tails)"', file=sys.stderr)
            return 1

        request = Announce.Request()
        request.formula = args.argument
        response = _call(
            node, Announce, 'epistemic_state/announce', request, args.timeout)
        if not _report(response):
            return 1

        print(
            f'announced: {response.num_worlds} worlds remain, '
            f'{response.num_designated} designated')
        return 0

    def _apply(self, node, args, ApplyAction):
        if not args.argument:
            print('apply needs an action named as the task names it', file=sys.stderr)
            return 1

        request = ApplyAction.Request()
        request.epistemic_action = args.argument
        request.observed_outcome = args.observed
        response = _call(
            node, ApplyAction, 'epistemic_state/apply_action', request, args.timeout)
        if not _report(response):
            return 1

        outcome = f' observing {response.outcome},' if response.outcome else ''
        print(
            f'applied {args.argument}:{outcome} {response.num_worlds} worlds, '
            f'{response.num_designated} designated')
        return 0

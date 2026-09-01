int16 REQUEST=1
int16 RESPONSE=2
int16 CONFIRM=3
int16 REJECT=4
int16 FEEDBACK=5
int16 FINISH=6
int16 CANCEL=7

int16 type
string node_id

string action
string[] arguments

bool success
float32 completion
string status

# What the performer observed while doing the action, meaningful on FINISH and
# empty everywhere else.
#
# `status` is a message for a human reading a log; this is a token the system
# acts on. A sensing action is one whose point is to find something out, and
# until now nothing carried the answer back: the executor learned that the
# action succeeded and nothing about what it revealed. An epistemic policy
# branches on exactly that, so the performer that did the looking has to be
# able to say what it saw.
#
# The token is the name of an outcome the planner planned for --- an event name
# of the action's model, not free text. A performer that reports one the policy
# does not list fails the branch rather than picking a plausible one, which is
# the same treatment an unanticipated observation gets anywhere else.
#
# Empty from an ordinary action, which is every classical performer: an action
# with one outcome has nothing to report, and the epistemic state derives the
# outcome itself when it can.
string outcome

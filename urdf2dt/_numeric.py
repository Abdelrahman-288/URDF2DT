"""Representation guards, separate from configurable research/FK tolerances.

Values retain the original algorithms' exact budgets; consolidation is not a
threshold retuning. Rendering cutoffs are intentionally presentation-local.
"""

from sys import float_info

REPRESENTATION_ATOL = 1.0e-9
AXIS_ROUND_OFF = 64 * float_info.epsilon
# Historical common-normal helper boundary, distinct from the solver roundoff gate.
COMMON_NORMAL_PARALLEL_ATOL = 1.0e-14

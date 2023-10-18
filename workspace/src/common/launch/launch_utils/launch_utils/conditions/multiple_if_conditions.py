"""MultipleIfConditions"""

from typing import Text, Iterable

from launch.conditions import evaluate_condition_expression
from launch.condition import Condition
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions


class MultipleIfConditions(Condition):
    """
    Encapsulates multiple if conditions to be evaluated when launching.
    These conditions take a string expression that is lexically evaluated as a
    boolean, but the expressions may consist of :py:class:`launch.Substitution`
    instances. Each expression must be true for the entire condition to be true.
    See :py:func:`evaluate_condition_expression` to understand what constitutes
    a valid condition expression.

    Parameter ``num`` is the sum of all the expressions. Will evaluate to True if
    the sum of the evaluate predicates is >= ``num``. If -1, all must be true.
    """

    def __init__(self, predicate_expressions: Iterable[SomeSubstitutionsType], *, num: int = -1) -> None:
        self.__predicate_expressions = [normalize_to_list_of_substitutions(
            expr) for expr in predicate_expressions]
        super().__init__(predicate=self._predicate_func)

        self.__num = num if num != -1 else len(predicate_expressions)

    def _predicate_func(self, context: LaunchContext) -> bool:
        return sum(evaluate_condition_expression(context, expr) for expr in self.__predicate_expressions) >= self.__num

    def describe(self) -> Text:
        """Return a description of this Condition."""
        return self.__repr__()

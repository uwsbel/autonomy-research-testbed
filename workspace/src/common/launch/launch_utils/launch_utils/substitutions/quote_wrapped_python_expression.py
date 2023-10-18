"""QuoteWrappedPythonExpression"""

from launch.substitutions import PythonExpression, LaunchConfiguration, TextSubstitution


class QuoteWrappedPythonExpression(PythonExpression):
    """Helper class that fixes a weird flaw in PythonExpression perform method where
    substituted LaunchConfiguration variables need to be wrapped in quotes."""

    def __init__(self, expression: 'SomeSubstitutionsType') -> None:
        super().__init__(expression)

        _expression = []
        for expr in self.expression:
            if isinstance(expr, LaunchConfiguration):
                _expression.append(TextSubstitution(text=("'")))
                _expression.append(expr)
                _expression.append(TextSubstitution(text=("'")))
            else:
                _expression.append(expr)

        # HACK
        vars(self)['_PythonExpression__expression'] = _expression

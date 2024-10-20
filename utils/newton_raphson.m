function [x, err, iters]  = newton_raphson(f, df, x0, tol, max_iters)
  % Find a root of f(x) = 0 using the Newton-Raphson method
  % Arguments:
  % - f: function handle
  %     the function describing the equation to solve
  % - df: function handle
  %     the derivative of f
  % - x0: float
  %     the initial guess to start the solver with
  % - tol: float
  %     the tolerance / precision at which computation stops
  %     default: 1e-9
  % - max_iters: int
  %     the maximum number of iterations to compute
  %     default: 100
  % Return values:
  % - x: float
  %     the found solution
  % - err: float
  %     the error in the function value
  % - iters: int
  %     the number of iterations actually run

  % Set default values for some arguments
  assert(nargin >= 3)
  if nargin < 5
    max_iters = 100;
  end
  if nargin < 4
    tol = 1e-9;
  end

  % Find the roots
  x = x0;
  for iters = 1:max_iters
    err = f(x);
    if abs(err) < tol
      break;
    end
    x = x - f(x) / df(x);
  end

end

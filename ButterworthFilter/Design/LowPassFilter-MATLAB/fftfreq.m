function f=fftfreq(npts,dt,alias_dt)
% returns a vector of the frequencies corresponding to the length
% of the signal and the time step.
% specifying alias_dt > dt returns the frequencies that would
% result from subsampling the raw signal at alias_dt rather than
% dt.
    
    
  if (nargin < 3)
      alias_dt = dt;
  end

  fmin = -1/(2*dt);
  df = 1/(npts*dt);

  f0 = -fmin;

  alias_fmin = -1/(2*alias_dt);
  f0a = -alias_fmin;
  
  ff = mod(linspace(0, 2*f0-df, npts)+f0,  2*f0)  - f0;
  fa = mod(                        ff+f0a, 2*f0a) - f0a;
  %  return the aliased frequencies
  f = fa;
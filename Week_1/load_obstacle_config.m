function obstacles = load_obstacle_config(config_name, p)
% load_obstacle_config.m
% =========================================================================
% Loads one of the three fixed obstacle configurations for reproducible
% testing across all optimization methods (Week 2+).
%
% INPUTS:
%   config_name – string: 'simple', 'moderate', 'hard', or 'none'
%   p           – parameter struct from params.m (contains p.configs.*)
%
% OUTPUT:
%   obstacles   – struct array with fields:
%       .type    – 'circle' or 'rectangle'
%       .cx, .cy – centre [m]
%       .r       – radius [m] (circles) or circumradius (rectangles)
%       .hw, .hh – half-width, half-height [m] (rectangles)
%       .phi     – orientation angle [rad] (rectangles)
%
% CONFIGURATIONS:
%   'simple'   : 1 obstacle, easy to clear (baseline)
%   'moderate' : 2 obstacles forming a corridor (robustness test)
%   'hard'     : 3 obstacles in slalom pattern (stress test)
%   'none'     : no obstacles (for debugging dynamics)
%
% USAGE:
%   p = params();
%   obstacles = load_obstacle_config('moderate', p);
%   [collides, ~] = check_collision(query, obstacles, p);
% =========================================================================

config_name = lower(strtrim(config_name));

switch config_name
    case 'simple'
        obstacles = p.configs.simple;
        fprintf('  Loaded obstacle configuration: SIMPLE (1 obstacle)\n');
        
    case 'moderate'
        obstacles = p.configs.moderate;
        fprintf('  Loaded obstacle configuration: MODERATE (2 obstacles)\n');
        
    case 'hard'
        obstacles = p.configs.hard;
        fprintf('  Loaded obstacle configuration: HARD (3 obstacles)\n');
        
    case 'none'
        obstacles = p.configs.none;
        fprintf('  Loaded obstacle configuration: NONE (0 obstacles)\n');
        
    otherwise
        error('load_obstacle_config: unknown config_name "%s". Valid: simple, moderate, hard, none.', ...
            config_name);
end

end

% To use the function, set the following initialization in the main script:
% hfig = figure;
% set(hfig,'KeyPressFcn',@set_input_by_key);
% setappdata(hfig, 'wl', 0);
% setappdata(hfig, 'wr', 0);
% setappdata(hfig, 'az', -30);
% setappdata(hfig, 'el', +20);
function set_utv_input_by_key(hfig, event)
    wl = getappdata(hfig, 'wl');
    wr = getappdata(hfig, 'wr');
    az = getappdata(hfig, 'az');
    el = getappdata(hfig, 'el');
    alpha = getappdata(hfig, 'alpha');
    switch event.Key
        case {'numpad5','5'}
            wl = 0;
            wr = 0;
        case {'numpad8','8'}
            wl = wl + 1;
            wr = wr + 1;
        case {'numpad2','2'}
            wl = wl - 1;
            wr = wr - 1;
        case {'numpad4','4'}
            wl = wl - 1;
            wr = wr + 1;
        case {'numpad6','6'}
            wl = wl + 1;
            wr = wr - 1;
        case {'uparrow','up'}
            el = el + 5;
        case {'downarrow','down'}
            el = el - 5;
        case {'leftarrow','left'}
            az = az + 5;
        case {'rightarrow','right'}
            az = az - 5;
    end
    setappdata(hfig, 'wl', wl);
    setappdata(hfig, 'wr', wr);
    setappdata(hfig, 'az', az);
    setappdata(hfig, 'el', el);
    setappdata(hfig, 'alpha', alpha);
end
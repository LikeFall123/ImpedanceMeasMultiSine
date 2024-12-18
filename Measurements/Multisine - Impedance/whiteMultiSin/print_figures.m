function print_figures(fig, name, width, height, res)
    set(fig, 'paperunits', 'inches', 'paperposition', [0 0 [width height]/res]);
    set(fig, 'renderer', 'painters')
    print(fig, strcat(name, '.svg'), '-dsvg');
    print(fig, strcat(name, '.png'), '-dpng', '-r300');
end
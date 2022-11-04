size = 10;
obs = 15;
num_of_agents = 5;

for k = 1:60
    %size = floor((k+9)/10) + 6;
    %percentage = floor((k+9)/10)/20;
    %obs = floor(size^2*percentage);
    num_of_agents = floor((k+9)/10);
    A = strings(size);
    A(:) = '.';
    SGmat = zeros(size);    
    xs = 0;
    ys = 0;
    xg = 0;
    yg = 0;
    count = 0;
    while count ~= obs
        o_x = randi(size);
        o_y = randi(size);
        if(A(o_x, o_y) == '.')
           A(o_x, o_y) = '@';
           count = count + 1;
        end
    end
    
    
    placed = 0;
    x = 0;
    y = 0;
    agents = zeros(num_of_agents, 4);
    while placed ~= num_of_agents
        xs = randi(size);
        ys = randi(size);
        xg = randi(size);
        yg = randi(size);  
        if((A(xs, ys) == '.') && (A(xg, yg) == '.') && SGmat(xs, ys) ~= -1 && SGmat(xg, yg) ~= 1 && ~isequal([xs, ys], [xg, yg]))
          agents(placed + 1, 1) = ys-1;
          agents(placed + 1, 2) = xs-1;
          agents(placed + 1, 3) = yg-1;
          agents(placed + 1, 4) = xg-1;
          SGmat(xs, ys) = -1;
          SGmat(xg, yg) = 1;
          placed = placed + 1;
        end      
    end
  
    fid = fopen(['testsAgents/test' num2str(k,'%02d') '.txt'], 'wt');
    fprintf(fid, '%d %d', [size, size]);
    fprintf(fid, '\n');
    for ii = 1:size
        fprintf(fid, '%s ', A(1:end-1, ii));
        fprintf(fid, '%s', A(end, ii));
        fprintf(fid, '\n');
    end
    fprintf(fid, '%d', num_of_agents);
    fprintf(fid, '\n');
    for ii = 1:num_of_agents
        fprintf(fid, '%d %d %d %d', agents(ii, :));
        fprintf(fid, '\n');
    end
    fclose(fid);
    
end




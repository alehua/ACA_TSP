clear;clc;
N = 15;
x = 4*rand(1,N)-2;
y = 4*rand(1,N)-2;
% x=[0.1681,0.4673,-0.3011,0.6811,-0.8644,-0.7134,-0.2060, -0.8642, 0.5387,-0.1939];
% y=[-0.0144,-0.1339,0.3610,-0.7007,0.2344,0.2937,0.8539,-0.8682,0.8964, -0.9031];
citys = [x',y'];
save_citys = citys;
citys = [citys;citys(1,:)];
[r,c] = size(citys);
Init_L = 0;
for i=2:r
    Init_L = Init_L+dist(citys(i-1,:),citys(i,:)');
end
figure(1)
plot(citys(:,1),citys(:,2))
title('Init Route')
hold on 
plot(citys(:,1),citys(:,2),'ro')
for i=1:N
   text(citys(i,1)+0.01,citys(i,2)+0.01,num2str(i));
end
hold off
%% 计算城市距离
DistanceCity=dist(save_citys,save_citys');

%% 初始化参数
City_num = N;
Ant_num = 5*N;
times = 200;                         %迭代次数
Q = 4;                              % 每一个蚂蚁所带信息素总量
eta = 1./DistanceCity;              % Eta为启发因子，这里设为距离的倒数
beta = 5;                            % 启发因子的重要程度
tao = zeros(City_num,City_num);     % Tao为信息素矩阵,最初值最大为1
for i=1:City_num
    for j=1:City_num
        if(j==i)
            continue
        else
            tao(i,j)=1;
        end
    end
end
alpha = 1.1;                        % 信息素的重要程度
der_info = 0.1;                     % 蚂蚁走过之后信息素增量   没用
dec_info = 0.7;                    % 每步信息素减少量
Table = zeros(Ant_num,City_num);    %存储一次迭代每只蚂蚁所走路径
std = 1:City_num;
best_route = cell(1,times);

%% 循环开始
for loop1 = 1:times
%初始化 所有蚂蚁初始所在位置
Init_position = zeros(Ant_num,1);
for i=1:Ant_num
    temp = randperm(City_num);
    Init_position(i) =  temp(1);
end

Table(:,1) = Init_position;   % 禁忌表第一列放置初始位置


for city=1:City_num-1  %每一个城市走一遍 
%% 第一次循环 所有蚂蚁走一步
for i=1:Ant_num
    % step 1计算走到下一个城市概率
    index = find(Table(i,:)==0);     %找到禁忌表第一个零所在位置
    p = ones(1,City_num-index(1)+1);
    temp_std = std;
    for k=1:index(1)-1               %将禁忌表中已经又得排除 
        temp_index = find(temp_std==Table(i,k));
        temp_std(temp_index)=[];
    end
    position_now = Table(i,index(1)-1);
    for j=1:City_num-index(1)+1
        p(j) = (tao(position_now,temp_std(j))^alpha)*(eta(position_now,temp_std(j))^beta);   
    end
    p_total = cumsum(p);
    p_next = p./p_total(end);
    % step 2轮盘赌
    p_disk = cumsum(p_next);   %CDF
    choose = rand;
    for t=1:City_num-index(1)+1
        if(p_disk(t)>choose)
            position_next = t;
            break
        end
    end
    Table(i,City_num+1-length(index)) = temp_std(t); 
    % 信息素在所有蚂蚁走完一次后跟新
end

end
%% 计算当前蚂蚁所走路径长度
%step4 
distance = zeros(Ant_num,1);
for i=1:Ant_num
    route = [Table(i,:),Table(i,1)];
    for j=1:City_num
       distance(i,1)=distance(i,1)+DistanceCity(route(j),route(j+1));
%        % 在这里添加信息素
%        tao(route(j),route(j+1)) = (1-dec_info)*tao(route(j),route(j+1)) + der_info;
%        if(tao(route(j),route(j+1))>1)
%            tao(route(j),route(j+1)) = 1;
%        end
    end  
end
% find the best
[distant,order] = sort(distance);
now_best = [ Table(order(1),:),distant(1)] ;                          %note 这里的1 改为迭代次数times
best_route{1,loop1} =  now_best;
%% 跟新信息素                这里是有问题的
%step 5 考虑信息素的挥发
for i=1:Ant_num
    route = [Table(i,:),Table(i,1)];   %是不是要考虑回到原点
    info = Q/distance(i);
    for j=1:City_num
       tao(route(j),route(j+1))= (1-dec_info)*tao(route(j),route(j+1))+DistanceCity(route(j),route(j+1))*info;
       if( tao(route(j),route(j+1))>1)
            tao(route(j),route(j+1)) = 1;
       else if( tao(route(j),route(j+1))<0)
                tao(route(j),route(j+1)) = 0.02; %留点余地
           end
       end
    end  
end


% 到这里完成一次循环
Table = zeros(Ant_num,City_num);  % 禁忌表清零
end

%% 画图
opti_dist = zeros(1,times);
for i=1:times
   opti_dist(i) = best_route{1,i}(end);  
end
figure(2)
plot(1:times,opti_dist)
title('最优距离')
figure(2)
result = best_route{1,times}(1,1:end-1);
finnal_city = zeros(City_num,2);
for i=1:N
   finnal_city(i,:) = save_citys(result(i),:);
end
finnal_city = [finnal_city;finnal_city(1,:)];
figure(3)
title('choose Route')
plot(finnal_city(:,1),finnal_city(:,2));
hold on 
plot(citys(:,1),citys(:,2),'ro')
for i=1:N
   text(citys(i,1)+0.01,citys(i,2)+0.01,num2str(i));
end
title('最后路径')
hold off














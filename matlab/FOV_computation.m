%% FOV computation for structured-light camera

lmin=0.6;
lmax=1.2;
delta_l=0.01;
num=(lmax-lmin)/delta_l;

distance=zeros(1,num);
width=zeros(1,num);
height=zeros(1,num);
for i=1:1:num
    distance(i)=lmin+i*delta_l;
    l=distance(i);
    
    wl1=tan(abs(-20.826)*pi/180)*l+0.1;
    wl2=tan(abs(28.826)*pi/180)*l-0.1;
    wl3=tan(22.3*pi/180)*l;
    wr1=tan(abs(26.826)*pi/180)*l-0.1;
    wr2=tan(abs(-18.826)*pi/180)*l+0.1;
    wr3=tan(22.3*pi/180)*l;
    w=min(wl1,min(wl2,wl3))+min(wr1,min(wr2,wr3));
    
    h1=tan(abs(18.32)*pi/180)*l*2;
    h2=tan(14.4*pi/180)*l*2;
    h=min(h1,h2);
    
    width(i)=w;
    height(i)=h;
    
    if abs(l-0.900)<0.01
        best_width=width(i);
        best_height=height(i);
    end
end

figure;
plot(distance,width);
hold on;
plot(distance,height);
axis([0.55,1.25,0.25,1]);
hold off;
best_width
best_height

0.72:0.45





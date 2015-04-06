function opcodemeshdemo_clean()

close all

% Very simple shape
% f = [1 2 3]'; v = [-1 0 0 ; 1 1 0 ; 1 -1 0]';
% [hit,d,trix,bary] = opcodemeshmex('intersect',t,1,1,[0 0 0.5]',[0 0 -1]');
 
% More complex shape
obj = read_obj('deer_bound.obj');
v = obj.vertices;
f = obj.objects(3).data.vertices;

% CLEAN: Reshape vertices
v = v';
f = f';
temp = v;
v(1,:) = v(3,:);
v(3,:) = temp(1,:);

% Add wall CAD model (2 large facets) after shifting and rotating object --
wallX1 = 3;
wallX2 = -3;
wallY1 = 3;
wallY2 = -3;
wallZ  = 3;

v(:,end+1:end+4) = [wallX1 wallX1 wallX2 wallX2;...
                    wallY1 wallY2 wallY1 wallY2;...
                    wallZ  wallZ  wallZ  wallZ];
f(:,end+1:end+2) = [size(v,2)-3 size(v,2)-3;...
                    size(v,2)-1 size(v,2)-2;...
                    size(v,2)-0 size(v,2)-0];
% -------------------------------------------------------------------------

figure, patch_display(struct('vertices',v','faces',f'))
daspect([1 1 1]),set(gca,'XDir','reverse'),set(gca,'ZDir','reverse')
xlabel('X-axis (mm)'),ylabel('Y-axis (mm)'),zlabel('Z-axis (mm)')

minb = min(v,[],2);
maxb = max(v,[],2);
%t = opcodemeshmex('create',v',f');
t = opcodemesh(v,f);

imgsize = 300;

x = (imgsize:-1:1) ./ imgsize .* (maxb(1) - minb(1)) + minb(1);
y = (1:imgsize) ./ imgsize .* (maxb(2) - minb(2)) + minb(2);
[X,Y] = meshgrid(x,y);
Y = flipud(Y);
Z = 1000*ones(size(X));

from = [X(:)+10 Y(:) -Z(:)]'
to =   [X(:)-10 Y(:) Z(:)]'

%[hit,d,trix,bary] = opcodemeshmex('intersect',t,from,to-from);
[hit,d,trix,bary] = t.intersect(from,to-from);
img_d = reshape(d,size(Z));
figure,imagesc(img_d);
figure,imshow(img_d);

bary(isnan(bary)) = size(f,2);
trix(trix==0)     = 1;
xvals =  bary(1,:).*v(1,f(1,trix)) + bary(2,:).*v(1,f(2,trix)) + (1-bary(1,:)-bary(2,:)).*v(1,f(3,trix));
figure, imshow(reshape(xvals,size(Z)),[min(xvals(:)) max(xvals(:))])

% You can update the vertices on the fly
v=v+0.2;
%opcodemeshmex('update',t,v');
t.update(v);

%[hit,d,trix,bary] = opcodemeshmex('intersect',t,from,to-from);
[hit,d,trix,bary] = t.intersect(from,to-from);
img_d = reshape(d,size(Z));
figure,imagesc(img_d);

% For correct values of d, normaize each column of the (to-from) 3xN matrix
[hit,d,trix,bary] = t.intersect(from,normc(to-from));
img_d = reshape(d,size(Z));
figure,imagesc(img_d);

% opcodemeshmex('delete',t);
% Unfriendly Alien
clear
a = zeros(100,100);
a(20,20:80) = ones(1,61);
a(80,20:80) = ones(1,61);
a(20:80,20) = ones(61,1);
a(20:80,60) = ones(61,1);

a(40,40:60) = ones(1,21);
a(60,40:60) = ones(1,21);
a(40:60,40) = ones(21,1);
a(40:60,60) = ones(21,1);

figure(1)
imagesc(a)
axis('square')

[U,S,V] = svd(a);

f = 1:4;
n = length(f);
d = [];
for i = 1:n
d(:,:,i) = U(:,1:f(i))*S(1:f(i),1:f(i))*V(:,1:f(i))';
end
figure()
subplot(2,2,1)
imshow(d(:,:,1));
subplot(2,2,2)
imshow(d(:,:,2));
subplot(2,2,3)
imshow(d(:,:,3));
subplot(2,2,4)
imshow(d(:,:,4));
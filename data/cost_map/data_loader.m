%**************************************************************************
% 
% Copyright (C) 2020, Jaein Lim and Panagiotis Tsiotras
% 
% 
% This library is free software; you can redistribute it and/or modify it under
% the terms of the GNU Lesser General Public License as published by the Free
% Software Foundation; either version 2.1 of the License, or any later version.
% 
% This library is distributed in the hope that it will be useful, but WITHOUT
% ANY WARRANTY;  without even the implied warranty of MERCHANTABILITY or
% FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
% for more details.
% 
% You should have received a copy of the GNU Lesser General Public License
% along with this library; if not, write to the Free Software Foundation, Inc.,
% 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
% 
%**************************************************************************/


%% data_loader_v1 saves the wavelet coefficient to .txt for tree construction

N = 7;
nV	= 2^(7);	% Total number of pixels in each row


%% Terrain map
load('map_q1632.mat');
fprintf('\nConfiguring environment map ...\t');
tic
X		= double(Img.cdata);
X		= imresize(X,[nV nV],'bilinear'); % resizing the image to [nV x nV] using interpolation
nColors = 256;
map		= gray(nColors); % create a [nColors x 3] matrix

Xrgb	= 0.2990*X(:,:,1)+0.5870*X(:,:,2)+0.1140*X(:,:,3);  % weighted average of RGB color 
Xdisp	= wcodemat(Xrgb, nColors); % rescale into nColors
Xtrue	= Xdisp;
toc

%% Initial wavelet decomposition and MR approximation
fprintf('Wavelet decomposition ...\t\t');
Y = 1-Xtrue/256; % Dark pixel to higher probability
tic																% Coarsest level of decomposition
[Cforig, Sz]	= wavedec2(Y, N, 'haar');								% Compute wavelet decomposition of original map
toc

pp=image(Xtrue);
axis equal
axis tight
colormap(map); 
print -dpng original_map.png

%% Write to file
% 
% fileID = fopen('map_wavelet.txt','w');
% fprintf(fileID,'%6.2f\n',Cforig);
% fclose(fileID);



    
    
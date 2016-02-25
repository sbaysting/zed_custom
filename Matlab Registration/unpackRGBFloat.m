function rgb = unpackRGBFloat(rgbfloatdata)
% Unpack RGB float data into separate color values
% rgbfloatdata - the RGB data packed into Nx1 floats
% rgb - Nx3 unpacked RGB values
%
% Author: Kevin Lai

mask = hex2dec('000000FF');
rgbf = typecast(rgbfloatdata,'uint32');

r = uint8(bitand(bitshift(rgbf,-16),mask));
g = uint8(bitand(bitshift(rgbf,-8),mask));
b = uint8(bitand(rgbf,mask));
rgb = [r g b];


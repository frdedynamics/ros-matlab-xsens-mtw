% 	Copyright (c) 2003-2016 Xsens Technologies B.V. or subsidiaries worldwide.
%	All rights reserved.

%	Redistribution and use in source and binary forms, with or without modification,
%	are permitted provided that the following conditions are met:

%	1.	Redistributions of source code must retain the above copyright notice,
%		this list of conditions and the following disclaimer.

%	2.	Redistributions in binary form must reproduce the above copyright notice,
%		this list of conditions and the following disclaimer in the documentation
%		and/or other materials provided with the distribution.

%	3.	Neither the names of the copyright holders nor the names of their contributors
%		may be used to endorse or promote products derived from this software without
%		specific prior written permission.

%	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
%	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
%	MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
%	THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
%	SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
%	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
%	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
%	TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
%	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


function varargout = plotSubplotQuat(time, quat, type, color)

% quat is the matrix containing nFrame X 4 components
% type is the name of the segment/joint

nFrame = size(quat, 1);
f = figure('Name','Xsens Quat Plot');
h_ax(1) = subplot(4,1,1);
h_l(1) = plot(time, quat(:,1), color(1));
xlabel('time(s)');
ylabel('Theta');
title(type);

h_ax(2) = subplot(4,1,2);
h_l(2) = plot(time, quat(:,2), color(2));
xlabel('time(s)');
ylabel('Epsilon x');

h_ax(3) = subplot(4,1,3);
h_l(3) = plot(time, quat(:,3), color(3));
xlabel('time(s)');
ylabel('Epsilon y');

h_ax(4) = subplot(4,1,4);
h_l(4) = plot(time, quat(:,4), color(4));
xlabel('time(s)');
ylabel('Epsilon z');

if nargout ~= 0
    varargout{1} = f;
    varargout{2} = h_ax;
    varargout{3} = h_l;
else
    linkaxes(h_ax,'x');
end
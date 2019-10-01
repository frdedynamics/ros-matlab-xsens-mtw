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


function varargout = plotInertialAcc(time, acc, type, color)

% angles is the matrix containing nFrame X 3 components
% type is the name of the joint
% eulerSequence is like (xzy)
angles = acc;
f = figure('Name','Xsens Acceleration Plot');
h_ax(1) = subplot(3,1,1);
h_l(1) = plot(time, angles(:,1), color(1));
xlabel('time(s)');
ylabel('[m/s^2]');
title(type);

h_ax(2) = subplot(3,1,2);
h_l(2) = plot(time, angles(:,2), color(2));
xlabel('time(s)');
ylabel('[m/s^2]');

h_ax(3) = subplot(3,1,3);
h_l(3) = plot(time, angles(:,3), color(3));
xlabel('time(s)');
ylabel('[m/s^2]');
if nargout == 0
    linkaxes(h_ax,'x')
    saveas(f, strcat(type,'MTi_results'),'fig');
else
    varargout{1} = f;
    varargout{2} = h_ax;
    varargout{3} = h_l;
end

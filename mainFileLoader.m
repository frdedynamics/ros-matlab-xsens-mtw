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


function [sdi, time] = mainFileLoader (filename)
    %%------- HELP
    %
    % This script allows the user to understand the step-wise procedure to load a file 
    %
    % The code is divided into two parts:
    %
    % 1) Set-up of the system and read some data
    %
    % 2) Event handler of the MT.
    %
    %%-------- IMPORTANT NOTES
    %
    % - For the code to work properly, make sure the code folder is your current directory in Matlab.
    %
    % - This code supports both 32 and 64 bits Matlab version.
    %
    % - The code requires xsensdeviceapi_com.dll to be registered in the Windows
    %   register (this is done automatically during the Xsens MT SDK installation)
    %
 
    %% Launching activex server
    try
        switch computer
        case 'PCWIN'
            h = actxserver('xsensdeviceapi_com32.IXsensDeviceApi');
        case 'PCWIN64'
            h = actxserver('xsensdeviceapi_com64.IXsensDeviceApi');
        otherwise
            error('CMT:os','Unsupported OS');
        end
    catch e
        fprintf('\n Please reinstall MT SDK or check manual,\n Xsens Device Api is not found.\n')
        rethrow(e);
    end
    fprintf( '\n ActiveXsens server - activated \n' );

    %% open the log file
    h.XsControl_openLogFile(filename);

    % getting number of MTs
    deviceID = cell2mat(h.XsControl_mainDeviceIds());
    num_MTs = length(deviceID);
    if num_MTs > 1
        fprintf('\n More than one device found, this script only uses the first one: %s\n',dec2hex(deviceID))
    else
        fprintf('\n Device found: %s\n',dec2hex(deviceID))
    end
    device = h.XsControl_device(deviceID);

    %% Tell the device to retain the data
    h.XsDevice_setOptions(device, h.XsOption_XSO_RetainRecordingData, 0);

    %% Load the log file and wait until it is loaded
    h.registerevent({'onProgressUpdated', @eventhandlerXsens});
    h.XsDevice_loadLogFile(device);
    fileLoaded = 0;
    while  fileLoaded == 0
        % wait untill maxSamples are arrived
        pause(.2)
    end
    fprintf('\nFile fully loaded\n')

    %% start data extracting
    % determine if device is Awinda station/dongle or a seperate device, in
    % case of dongle or station, find a MTw.
    if h.XsDeviceId_isWirelessMaster(deviceID)
        childDevices = h.XsDevice_children(device);
        device = childDevices{1};
        deviceID = h.XsDevice_deviceId(device);
        fprintf('\n Device found for data extracting: %s\n',dec2hex(deviceID))
    end
    % get total number of samples
    nSamples = h.XsDevice_getDataPacketCount(device);
    % allocate space
    sdi(1:nSamples) = struct('dq',zeros(4,1),'dv',zeros(3,1));
    time = zeros(1,nSamples); % if time available, otherwise packet counter
    hasPacketCounter = false;
    hasTimeData = false;
    hasSdiData = false;
    readSample = 0;
    % for loop to extract the data
    for iSample = 1:nSamples
        % get data packet
        dataPacket =  h.XsDevice_getDataPacketByIndex(device,iSample);
        % check if dataPacket is a data packet
        if dataPacket
            readSample = readSample+1;
            % see if data packet contains certain data
            if h.XsDataPacket_containsSdiData(dataPacket)
                hasSdiData = true;
                % extract data, data will always be in cells
                sdiData = cell2mat(h.XsDataPacket_sdiData(dataPacket));
                sdi(readSample).dq = sdiData(1:4);
                sdi(readSample).dv = sdiData(5:7);
            end
            if h.XsDataPacket_containsSampleTimeFine(dataPacket)
                hasTimeData = true;
                time(readSample) = h.XsDataPacket_sampleTimeFine(dataPacket);
            elseif h.XsDataPacket_containsPacketCounter(dataPacket)
                hasPacketCounter = true;
                time(readSample) = h.XsDataPacket_packetCounter(dataPacket);
            end
        end
    end
    % clean up of allocated space
    time(readSample+1:end) = [];
    sdi(readSample+1:end) = [];

    %% show delta velocity data
    % if not time or packetCounter is available, create own for plotting
    if ~(hasTimeData || hasPacketCounter)
        time = 1:readSample;
        hasPacketCounter = true;
    end
    % if sdi data is present, show it
    if hasSdiData
        figure('Name','MT SDK example mainFileLoader')
        dv = [sdi.dv];
        if hasPacketCounter
            plot(time-time(1),dv);
            xlabel('PacketCounter (n)')
        else
            plot((time-time(1))/1e4,dv);
            xlabel('Time (s)')
        end
        ylabel('dv (m/s)')
        title(['Delta velocity of device ' dec2hex(deviceID)])
    else
        fprintf('No sdi data present in file\n')
    end

    %% close port and object
    h.XsControl_close();

    delete(h); % release COM-object
    clear h;

%% event handling
    function eventhandlerXsens(varargin)
        % device pointer is zero for progressUpdated
        devicePtr = varargin{3}{1};
        % The current progress
        currentProgress = varargin{3}{2};
        % The total work to be done
        total = varargin{3}{3};
        % Identifier, in the case the file name which is loaded
        identifier = varargin{3}{4};
        if currentProgress == 100
            fileLoaded = 1;
        end
    end
end

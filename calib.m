classdef calib
        %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    methods(Static)

        function msg = extractFrames(videoDirectory)
            %extract frames at regular intervals from all videos so the
            %frames are uniform
            vidList = dir(fullfile(videoDirectory, '*.mp4'));
            T = struct2table(vidList);
            sortedT = sortrows(T, 'name');
            vidList = table2struct(sortedT);
            vidNames = {vidList.name};
            framedir = fullfile(videoDirectory,'frames');
            if ~exist(framedir,"dir")
                mkdir(framedir)
            end
            for i=1:length(vidNames)
                vid2readIn = fullfile(videoDirectory,vidNames(i));
                % import the video file 
                obj = VideoReader(vid2readIn); 
                vid = read(obj); 
                frames = obj.NumFrames; 
                
                % file format of the frames to be saved in 
                format ='.jpg'; 
                
                % reading and writing the frames  
                for x = 1:20:frames 
                    file_name = strcat('cam',num2str(i),'_frame',num2str(x)); 
                    imgName = fullfile(videoDirectory,'frames',file_name) ; 
                    Vid = vid(:, :, :, x);    
                    % exporting the frames 
                    imwrite(Vid, imgName,'jpg'); 
                end
                
            end
            msg = "frames extracted from videos";
            
        end



    end
end
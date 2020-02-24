clear;
clc;
close all;

det_files = dir('data/dets/*.txt');
gt_files  = dir('data/gts/*.txt');

dets = [];
gts  = [];

A = zeros(720,1280);
figure(1);
hold on
for i = 1:size(det_files,1)
    dets = [];
    gts  = [];
    figure(1);
    imshow(A);

    dets = read_bboxes(strcat(det_files(i).folder, '/', det_files(i).name), 'dets');
    gts  = read_bboxes(strcat(gt_files(i).folder, '/', gt_files(i).name), 'gts');
    
    l = 1;
    e = [];
    d = [];
    for j = 1:size(dets)
        rec_a = str2double(dets(j,:));
        for k = 1:size(gts)
            rec_b = str2double(gts(k,:));
            if rectint(rec_a,rec_b) > 0
                figure(1);
                rectangle('Position',rec_a,'Edgecolor', 'r');
                rectangle('Position',rec_b,'Edgecolor', 'g');
                e(l) = abs((rec_b(1) + rec_b(3)/2) - (rec_a(1) + rec_a(3)/2));
                d(l) = rec_b(1) + rec_b(3)/2;
                l = l + 1;
            end
        end
        figure(2);
        grid on
        hold on
        plot(d,e,'*');
    end
    pause()
end

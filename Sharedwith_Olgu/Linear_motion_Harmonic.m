clc 
clear
clc


% for r = 1:1
r=1;
    load_system('Simulink_model_Imbalances.slx');
    simIn = Simulink.SimulationInput('Simulink_model_Imbalances.slx');
    simIn = simIn.setBlockParameter("Simulink_model_Imbalances/Constant2","Value","r");
    out = sim(simIn);


    Force=out.Moment.signals.values;
    Force_time = Force(1,:,:);
    F=Force_time(1,:);
    RPMT=out.RPM_t.signals.values;
    Time=out.tout;

    Moment=out.Signal.signals.values;
    Moment_time = Moment(1,:,:);
    T=Moment_time(1,:);

F=F';   
T=T';
% Save T vector to a text file
filename = 'T.txt';
% Ensure column vector
T = T(:);
% Write as plain text, one value per line with high precision
fid = fopen(filename, 'w');
if fid == -1
    error('Cannot open file %s for writing.', filename);
end
fprintf(fid, '%.10e\n', T);
fclose(fid);
% Save F vector to a text file
filename = 'F.txt';
% Ensure column vector
F = F(:);
% Write as plain text, one value per line with high precision
fid = fopen(filename, 'w');
if fid == -1
    error('Cannot open file %s for writing.', filename);
end
fprintf(fid, '%.10e\n', F);
fclose(fid);


RPMRate= out.RateofRotation.signals.values;
% Save RPMRate and RPMT to text files
% Ensure column vectors
RPMRate = RPMRate(:);
RPMT = RPMT(:);

fid = fopen('RPMRate.txt','w');
if fid == -1
    error('Cannot open file RPMRate.txt for writing.');
end
fprintf(fid, '%.10e\n', RPMRate);
fclose(fid);

fid = fopen('RPMT.txt','w');
if fid == -1
    error('Cannot open file RPMT.txt for writing.');
end
fprintf(fid, '%.10e\n', RPMT);
fclose(fid);



%     for k = 1:150
%         % each segment uses 100000 samples: segment k uses indices (k-1)*10000+1 : k*10000
%         idxStart = (k-1)*20000 + 1;
%         idxEnd = k*20000;
%         if idxEnd > length(F)
%             idxEnd = length(F);
%         end
%         xseg = F(idxStart:idxEnd);
%         fs = 1000;
%         [Pxx, f] = pwelch(xseg, [], [], [], fs);
%         avg_RPMT = mean(RPMT(idxStart:idxEnd));
%         PSD_mat(k,:) = Pxx.';
%         frq_mat(k,:) = f.';
%         RPM_avg(k,1) = avg_RPMT;
%     end
%     waterfall(frq_mat, RPM_avg*ones(1,size(frq_mat,2)), PSD_mat)
%     xlabel('Frequency (Hz)')
%     ylabel(' RPM')
%     zlabel('PSD')
%     title('Waterfall Plot of PSD vs Frequency and RPM')
%     xlim([0 250])
%         % Increase font size of axis tick labels and axis labels/titles
%     ax = gca;
%     ax.FontSize = 18;           % tick labels
%     ax.FontWeight = 'bold';
%     ax.LineWidth = 1;
%     % Increase labels' font size (in case they were set earlier)
%     xlabelHandle = get(ax,'XLabel');
%     ylabelHandle = get(ax,'YLabel');
%     zlabelHandle = get(ax,'ZLabel');
%     titleHandle = get(ax,'Title');
%     set([xlabelHandle,ylabelHandle,zlabelHandle,titleHandle], 'FontSize', 18, 'FontWeight','bold');
%     % If using colorbar, increase its font size too
%     cb = colorbar('peer',ax);
%     cb.FontSize = 18;
%     cb.FontWeight = 'bold';
% 
% 
%     % Increase tick label font size (numbers on the axes)
%     ax = gca;
%     ax.FontSize = 20;    % make axis numbers larger
%     ax.FontWeight = 'bold';
%     % Also increase tick length for visibility
%     ax.TickLength = [0.02 0.02];
%     % Ensure tick labels are updated (in case of custom tick formatting)
%     set(ax, 'XTickLabelMode', 'auto', 'YTickLabelMode', 'auto', 'ZTickLabelMode', 'auto');
% 
%         % Increase font size of axis tick labels and axis labels/titles
%         ax = gca;
%         ax.FontSize = 14;           % tick labels
%         ax.FontWeight = 'bold';
%         ax.LineWidth = 1;
%         % Increase labels' font size (in case they were set earlier)
%         xlabelHandle = get(ax,'XLabel');
%         ylabelHandle = get(ax,'YLabel');
%         zlabelHandle = get(ax,'ZLabel');
%         titleHandle = get(ax,'Title');
%         set([xlabelHandle,ylabelHandle,zlabelHandle,titleHandle], 'FontSize', 16, 'FontWeight','bold');
%         % If using colorbar, increase its font size too
%         cb = colorbar('peer',ax);
%         cb.FontSize = 12;
%         cb.FontWeight = 'bold';
% 
%      hold on
% end
% hold off

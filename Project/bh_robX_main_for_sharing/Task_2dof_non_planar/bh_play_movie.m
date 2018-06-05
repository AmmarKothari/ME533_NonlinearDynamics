function bh_play_movie

    p = mfilename('fullpath');
    [folder,name,ext] = fileparts(p);

    try
          winopen([folder,filesep,'bh_2dof_NP_CONTROLLED.mp4']);
 
    catch
          winopen([folder,filesep,'bh_2dof_NP_CONTROLLED.avi']);
    end
end


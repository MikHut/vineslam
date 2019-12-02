function ang_normalized = NormalizeAng(ang)
    % this function normalizes an angle from [-pi,pi]

    ang_normalized = ang;
    while ang_normalized > pi()
      ang_normalized = ang_normalized - 2*pi();
    end
    while ang_normalized <= -pi
      ang_normalized = ang_normalized + 2*pi();
    end  

end    
function success = check_message(reading)
%CHECK_SIZE Summary of this function goes here
%   Detailed explanation goes here
len = size(reading);
success = false;
if len(1) ~= 0
    success = true;
end
end


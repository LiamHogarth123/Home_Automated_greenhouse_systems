function ReadArduino ()
    a = arduino('com4', 'uno');
    value = readDigitalPin(a, 'D2');
    if value == 1
        setEstopStatus(1);
%         tell ardino to set red light on
    end
end
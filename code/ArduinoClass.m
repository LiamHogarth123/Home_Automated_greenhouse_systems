classdef ArduinoClass
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        buttonPress;
        unlocked_mode;
        a
    end

    methods
        function self = ArduinoClass()
            
            self.a = arduino('com4', 'uno');
            self.buttonPress;
            self.unlocked_mode; 
            writeDigitalPin(self.a, 'D6', 1);
        end
    end

    methods (Static)
       

        function state = ReadArduino (self, state)
            value = readDigitalPin(self.a, 'D3');
            
            if isempty(state)
                state = 0;
            end
            if value == 1
                if (state == 0 || state == 3)
                    setEstopStatus(1);
                    writeDigitalPin(self.a, 'D4', 1);
                    state = 1;
                    writeDigitalPin(self.a, 'D6', 0);
                    writeDigitalPin(self.a, 'D5', 0);
                else 
                    writeDigitalPin(self.a, 'D5', 1);
                    state = 2;
                    writeDigitalPin(self.a, 'D4', 0);
                end
            end
            button2 = readDigitalPin(self.a,'D2');
            if (button2 == 1 && state == 2)
                setEstopStatus(0);
                %tell ardino to set green light on
                writeDigitalPin(self.a, 'D6', 1);
                writeDigitalPin(self.a, 'D5', 0);
                state = 0;
         
            end

        end

        function ReadWaterContent()
            
        end


    end
end
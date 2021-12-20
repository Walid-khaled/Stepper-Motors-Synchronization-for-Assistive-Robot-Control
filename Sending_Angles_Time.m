arduinoCom=serial('COM6','BAUD', 9600);
fopen(arduinoCom); %open the port
while true
a= input('Insert angles and Coordinated Time:');
if (length(a) == 7)
    fwrite(arduinoCom,num2str(a))
    pause(2)
    fscanf(arduinoCom)
elseif (a == 1 | a == 2 | a == 3 | a == 4 | a == 5 | a == 6)
    fwrite(arduinoCom,string(a))
    pause(2)
    fscanf(arduinoCom)
elseif (a == 0)
    break
else
    disp("Invalid input: Please Enter 6 Joints Angles and Coordinated Time")
end
end
fclose(arduinoCom)

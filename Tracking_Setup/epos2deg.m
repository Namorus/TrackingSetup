function[output_deg]=epos2deg(input_epos)

gearRatio=12167.0/64.0;
output_deg=input_epos*360/(gearRatio*2000.0);

end

function [kdx1, kdy1, kdx2, kdy2, kdx3, kdy3] = readDragCo()

    filename = "dragCo.csv";
    M = csvread(filename, 0, 0);
    kdx1 = M(1,1);
    kdy1 = M(1,2);
    kdx2 = M(1,3);
    kdy2 = M(1,4);
    kdx3 = M(1,5);
    kdy3 = M(1,6);
end
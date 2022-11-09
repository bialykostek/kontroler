out = "/****** NN DATA ********/" + newline;
out = out + "const int NNil = " + length(x1_step1.xoffset) + ";" + newline;
out = out + "const int NNol = " + length(y1_step1.xoffset) + ";" + newline;
out = out + "const int NNl1l = " + length(IW1_1(:, 1)) + ";" + newline;
out = out + "float NNinput[" + length(x1_step1.xoffset) + "];" + newline;
out = out + "float NNoutput[" + length(y1_step1.xoffset) + "];" + newline;

out = out + "float x1_step1_xoffset[" + length(x1_step1.xoffset) + "] = {";
first = true;
for i=1:length(x1_step1.xoffset)
    if first
        first = false;
    else
        out = out + ",";
    end
    out = out + sprintf("%.8f",x1_step1.xoffset(i));
end
out = out + "};" + newline;

out = out + "float x1_step1_gain[" + length(x1_step1.gain) + "] = {";
first = true;
for i=1:length(x1_step1.gain)
    if first
        first = false;
    else
        out = out + ",";
    end
    out = out + sprintf("%.8f",x1_step1.gain(i));
end
out = out + "};" + newline;

out = out + "float x1_step1_ymin = " + x1_step1.ymin + ";" + newline;

out = out + "float b1[" + length(b1) + "] = {";
first = true;
for i=1:length(b1)
    if first
        first = false;
    else
        out = out + ",";
    end
    out = out + sprintf("%.8f",b1(i));
end
out = out + "};" + newline;

out = out + "float IW1_1[" + length(IW1_1(:, 1)) + "][" + length(x1_step1.gain) + "] = {";
first = true;

for i=1:length(IW1_1(:, 1))
    if first
        first = false;
    else
        out = out + ",";
    end
    firstx = true;
    out = out + "{";
    for k=1:length(IW1_1(1, :))
        if firstx
            firstx = false;
        else
            out = out + ",";
        end
        out = out + sprintf("%.8f",IW1_1(i, k));
    end
    out = out + "}";
end
out = out + "};" + newline;

out = out + "float b2[" + length(b2) + "] = {";
first = true;
for i=1:length(b2)
    if first
        first = false;
    else
        out = out + ",";
    end
    out = out + sprintf("%.8f",b2(i));
end
out = out + "};" + newline;

out = out + "float LW2_1[" + length(y1_step1.gain) + "][" + length(b1) + "] = {";
first = true;
for i=1:length(LW2_1(:, 1))
    if first
        first = false;
    else
        out = out + ",";
    end
    firstx = true;
    out = out + "{";
    for k=1:length(LW2_1(1, :))
        if firstx
            firstx = false;
        else
            out = out + ",";
        end
        out = out + sprintf("%.8f",LW2_1(i, k));
    end
    out = out + "}";
end
out = out + "};" + newline;

out = out + "float y1_step1_ymin = " + y1_step1.ymin + ";" + newline;

out = out + "float y1_step1_gain[" + length(y1_step1.gain) + "] = {";
first = true;
for i=1:length(y1_step1.gain)
    if first
        first = false;
    else
        out = out + ",";
    end
    out = out + sprintf("%.8f",y1_step1.gain(i));
end
out = out + "};" + newline;

out = out + "float y1_step1_xoffset[" + length(y1_step1.xoffset) + "] = {";
first = true;
for i=1:length(y1_step1.xoffset)
    if first
        first = false;
    else
        out = out + ",";
    end
    out = out + sprintf("%.8f",y1_step1.xoffset(i));
end
out = out + "};" + newline;

out = out + "/****** END OF NN DATA ******/";

disp(out);

fid = fopen('nn.h','wt');
fprintf(fid, out);
fclose(fid);

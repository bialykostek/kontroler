function [y1] = myNeuralNetworkFunction(x1)
%MYNEURALNETWORKFUNCTION neural network simulation function.
%
% Auto-generated by MATLAB, 08-Nov-2022 14:40:49.
%
% [y1] = myNeuralNetworkFunction(x1) takes these arguments:
%   x = Qx21 matrix, input #1
% and returns:
%   y = Qx5 matrix, output #1
% where Q is the number of samples.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.xoffset = [0.245138888888889;0.369888888888889;0.245138888888889;0.369888888888889;0.32544;0.13775;0.0525;0.168957718368858;0.189140903587907;0.396549286990268;0.370577538712667;0.370577538712667;0.370577538712667;0.370577538712667;0.370577538712667;0.00566;0.0166666666666667;-0.216666666666667;-0.266666666666667;0.266666666666667;0.161111111111111];
x1_step1.gain = [3.99866711096301;8.20512820512821;3.99866711096301;8.20512820512821;4.53885257806826;5.4421768707483;4.32900432900433;2.69087165189704;3.19496022509305;14.2799666072263;7.7417966752017;7.7417966752017;7.7417966752017;7.7417966752017;7.7417966752017;3.92788404886288;3;1.21212121212121;1.21212121212121;2.92682926829268;4];
x1_step1.ymin = -1;

% Layer 1
b1 = [-1.0821698746316892326;-0.37631446814251223731;0.0400985656239818411;-0.18656133811644773357;0.28925992288046464829;0.048812418504702602406;-0.026856427878541977439;-0.66798721902318414223;-1.1854227675326771685;1.6084770946363540212];
IW1_1 = [0.37049313986441223534 -0.20188913631251093062 0.036771146550490421689 0.52355343202708726214 -0.13988714738587909592 -0.024454681753208827588 -0.15517387505638616241 0.63926280739340801684 0.70784774541786876778 -0.1642365950045449885 0.5739172184260806775 0.019093248720206051211 0.29489626767085158621 0.058513481925666438466 0.037789136631279918177 -0.069625581082573120839 -0.19067376451659256986 0.0089504715968284076499 0.17094640752202558409 0.14703545186470917683 0.073332584947723461477;0.053477292933499558092 0.54522450342897466058 -0.008672783755880460535 -0.45951316676008391315 -0.046587921564074999403 -0.047556525053667517144 -0.033664264564788895506 0.30788668544650893422 0.37069906594540724676 0.063717027518053442892 0.36254691364759550343 0.0066864098569951527229 0.35540499337415693004 0.069531274138809912033 0.036424391861355084887 -0.061276321193111042696 -0.82515236208053854838 0.18039156986041479658 0.25390369447792338065 0.044646496782674072179 0.030958396712459930467;0.016221381437028514066 0.046535435806103087053 0.0036684828162619250586 -0.050509772920681601793 -0.028159923852662794436 0.0031506790769428513754 0.0087838435547714539819 0.049751423289207868939 0.006592711511067894832 -0.00769101719379920723 -0.020063824072099511325 0.0028863296686191817 -0.070913809151579035306 -0.0090922388143615268802 0.015409163301760318149 0.0048476784460476790573 -0.099596891094966763514 -0.0898573901055390728 -0.01968815213385995827 -0.03532956681933011045 -0.50972174808994918926;0.13355320947654411579 -0.051899101916865357509 -0.028168704775796825357 0.047400531795016576098 0.4461192129470967771 0.23871790446831045451 -0.21540580055964758666 -0.3250848730408864018 0.32027864777399511764 -0.30228149356720546148 0.023527275577396419143 0.011848148921544517098 -0.039350441134474070659 -0.21613663313909151076 -0.24458623084195627029 -0.38225460061613653062 0.35601476205760129723 0.52839922860503463209 0.17858566387187960189 0.42622966305777004736 -0.22274279930427259844;-0.40651869756014386015 0.14635881918793447465 0.42848774414594675841 -0.1113756037956707412 -0.085102914164437173938 -0.038879796086081247486 -0.0039611362641938981793 0.17606923728923823402 0.080466579980905394986 0.064104368418046536737 0.037304392969128186142 -0.023378791962744220612 -0.041946888452922610757 0.064640430390095510171 0.10105031413249758454 0.020516383275368806766 0.75618976226820922015 0.27893585336367987537 0.68277684066753718817 0.55916778681778245286 -0.094878777854132109737;-0.61704497623748710833 -0.13548098411929390683 0.63563229739679572017 0.13906238158196981836 -0.049942241481879520726 -0.0071940457240816605561 0.025791540913668257323 0.045566012440303726994 -0.0044974258424552975705 -0.024582347548431419254 -0.014320790716679736299 0.0012395839153808031058 -0.061593853124951435107 0.0065454242052616030609 0.011684578229706144015 0.019969696408948467425 0.42480532724680980516 -0.32205664695061642089 -0.082597608486796009197 -0.44506110542229793126 -0.022653041659710205363;0.85917291548928553802 0.021871865089660959963 -0.90739622731508173814 -0.040515828644195020503 0.02515245530849496633 -0.011655177563164881988 -0.0058921269430100035522 -0.016290556180072635062 0.042959710093395407637 0.070863385748167195821 0.089148455322726347094 0.0054404488098831124687 0.19229807097110715453 0.001840340604375311287 -0.016983108444002029819 -0.0152501931305283802 -0.02236249283871577162 0.38182094701253377345 -0.2203294397247599623 -0.084509320460957049193 -0.15803628596061528033;-0.82316733482326420734 -0.72954040170208733773 0.36131915977059697598 0.67722034061110680803 0.03733430280021253489 -0.06456345554302969636 0.029087141248647977593 -0.32080092839057916532 -0.38201548309713795337 0.23253845033448289348 -0.0014270731872901218362 0.0017228593130242577519 0.26509119926033974934 -0.11673182095961072424 -0.11785605020829170719 0.15634582843463903923 0.19478450228165025693 -0.10920313641794501813 -0.15958959695579871818 -0.0081501141865385866303 -0.076153755483455418118;-0.22998880064009133428 0.25274210997737112105 0.4844878936159559002 -0.27483607358830131062 0.019486849872038709136 0.0022756301827195424617 0.079541709971851851102 0.2544914044516568552 0.06747092470735215064 -0.14921185978339734923 0.098706960192423698341 0.27381244022142603445 0.18867551591133188693 0.0054972416513767051113 -0.19156196493252281021 0.032805709540670552338 0.97774466985476571335 0.048075084726175519589 0.11851970880088791294 0.45121382551564664931 -0.22389981829475041897;0.34980642325472760534 -0.094656835495991520357 -0.11166449414657560191 0.36005684300021295607 0.4649664093333629733 0.000874203457905491782 -0.29845008223837010686 -0.62563872742528037207 -0.052647858352484692979 -0.32009988762205643376 -0.11465801814407231007 -0.18868742602134558339 0.26172212034486053156 0.36992843173571565707 0.024634328288687497993 -0.12591461827179439847 0.028986727814074405435 0.51626114805907674654 0.22759340154653159849 0.70079505797327301675 -0.14606570334639151776];

% Layer 2
b2 = [-0.065045465426654375252;-0.97979722614937425629;0.98227164816385392765;0.06870609563440464862;0.27511331594807830747];
LW2_1 = [0.041185025543954617566 -0.27522546473435766101 -0.37762399551996544966 0.11010993383398040568 0.53323507608149778303 0.75186734612423378188 0.49590944773648654165 0.06794546324510780666 0.2206328045624787737 0.088359712162534617685;0.40681377747602143202 -1.0577753106392573379 -0.1902769556687074759 -0.25901074729537892916 -0.26082608372252447104 -0.71584006528878252329 1.3998077392136747399 -0.78802221459682630922 -0.15175523791402373286 0.49715620743913091495;-0.40962445539966585439 1.0606910803012130629 0.18893346302012578586 0.25964865712844864554 0.26034580978620902592 0.71662828624919772125 -1.4024539970120051269 0.78926686274330326576 0.15464326803018757506 -0.49551587107984795821;0.1581608913747400702 -0.24673381291234458557 0.030282741447970919213 0.01184741440101556853 0.41048924355116167551 -1.4467556415524334135 -0.32716710372229135251 0.088046378675282893633 0.30620890172729919865 0.1917664147785141382;0.067997972221689184646 -0.12868106016939634406 -1.8342706422221066731 -0.073365112137789803559 -0.096974563102373220747 -0.13495038794010505434 -0.26092274504877616126 -0.2568601286835880404 -0.13023262412502861762 -0.47191997659732365378];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = [3;1.21212121212121;1.21212121212121;2.92682926829268;4];
y1_step1.xoffset = [0.0166666666666667;-0.216666666666667;-0.266666666666667;0.266666666666667;0.161111111111111];

% ===== SIMULATION ========
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

% Dimensions
Q = size(x1,1); % samples

% Input 1
x1 = x1';
xp1 = mapminmax_apply(x1,x1_step1);

% Layer 1
a1 = tansig_apply(repmat(b1,1,Q) + IW1_1*xp1);

% Layer 2
a2 = repmat(b2,1,Q) + LW2_1*a1;

% Output 1
y1 = mapminmax_reverse(a2,y1_step1);
y1 = y1';
end

% ===== MODULE FUNCTIONS ========

% Map Minimum and Maximum Input Processing Function
function y = mapminmax_apply(x,settings)
y = bsxfun(@minus,x,settings.xoffset);
y = bsxfun(@times,y,settings.gain);
y = bsxfun(@plus,y,settings.ymin);
end

% Sigmoid Symmetric Transfer Function
function a = tansig_apply(n,~)
a = 2 ./ (1 + exp(-2*n)) - 1;
end

% Map Minimum and Maximum Output Reverse-Processing Function
function x = mapminmax_reverse(y,settings)
x = bsxfun(@minus,y,settings.ymin);
x = bsxfun(@rdivide,x,settings.gain);
x = bsxfun(@plus,x,settings.xoffset);
end
var start = 350;
var end = 5500;

$.get("http://jkostecki.ddns.net:8080/log_08_11_14_21_05.txt", (inp) => {

    inp1 = inp.split('\n')
    output = []
    for(var i=start; i < end; i++){
        try{
        inp1[i] = inp1[i].split(",");
        if(inp1[i].length == 33){
            output.push([]);
            for(var k = 0; k <= 32; k++){
                 output[output.length-1].push(parseFloat(inp1[i][k]));
            }
        }
        }catch(e){
        }
    }
    var final = [];
    for(var i=1; i<output.length; i++){
    
        try{
            final.push([]);
            final[final.length-1].push((output[i][6]+180)/360);
            final[final.length-1].push((output[i][7]+180)/360);
            final[final.length-1].push((output[i-1][6]+180)/360);
            final[final.length-1].push((output[i-1][7]+180)/360);
            final[final.length-1].push(output[i][13]/25000);
            final[final.length-1].push(output[i][11]/40);
            final[final.length-1].push(output[i][16]/20);
    
            var tmp = output[i][23];
            tmp *= -1;
            tmp += Math.PI;
            if(tmp > 2*Math.PI){
                tmp -= 2*Math.PI;
            }
            tmp /= 2*Math.PI;
            final[final.length-1].push(tmp);
    
            tmp = output[i][24];
            tmp *= -1;
            tmp += Math.PI;
            if(tmp > 2*Math.PI){
                tmp -= 2*Math.PI;
            }
            tmp /= 2*Math.PI;
            final[final.length-1].push(tmp);
    
            final[final.length-1].push((output[i][25]+Math.PI)/2/Math.PI);
            
            tmp = output[i][26];
            tmp *= -1;
            tmp += Math.PI;
            if(tmp > 2*Math.PI){
                tmp -= 2*Math.PI;
            }
            tmp /= 2*Math.PI;
            final[final.length-1].push(tmp);
    
            tmp = output[i][27];
            tmp *= -1;
            tmp += Math.PI;
            if(tmp > 2*Math.PI){
                tmp -= 2*Math.PI;
            }
            tmp /= 2*Math.PI;
            final[final.length-1].push(tmp);
    
            tmp = output[i][28];
            tmp *= -1;
            tmp += Math.PI;
            if(tmp > 2*Math.PI){
                tmp -= 2*Math.PI;
            }
            tmp /= 2*Math.PI;
            final[final.length-1].push(tmp);
    
            tmp = output[i][29];
            tmp *= -1;
            tmp += Math.PI;
            if(tmp > 2*Math.PI){
                tmp -= 2*Math.PI;
            }
            tmp /= 2*Math.PI;
            final[final.length-1].push(tmp);
    
            tmp = output[i][30];
            tmp *= -1;
            tmp += Math.PI;
            if(tmp > 2*Math.PI){
                tmp -= 2*Math.PI;
            }
            tmp /= 2*Math.PI;
            final[final.length-1].push(tmp);
    
            final[final.length-1].push(output[i][31]/500);
    
            final[final.length-1].push((output[i-1][0]-60)/60);
            final[final.length-1].push((output[i-1][1]-60)/60);
            final[final.length-1].push((output[i-1][2]-60)/60);
            final[final.length-1].push((output[i-1][3]-60)/60);
            final[final.length-1].push(output[i-1][4]/180);
    
            final[final.length-1].push((output[i][0]-60)/60);
            final[final.length-1].push((output[i][1]-60)/60);
            final[final.length-1].push((output[i][2]-60)/60);
            final[final.length-1].push((output[i][3]-60)/60);
            final[final.length-1].push(output[i][4]/180);
            
            
        }catch(e){}
    }
    var out = "";
    for(var i = 0; i< final.length; i++){
        for(var k=0; k < final[i].length; k++){
            if(k!=0){
                out += ",";
            }
            out += final[i][k];
        }
        out += "\n";
    }
    $("#aiout").val(out);
});

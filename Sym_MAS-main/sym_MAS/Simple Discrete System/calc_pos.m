pos=2;
if(rem(pos,n_y)==0)
    pos_y = 5;
else
    pos_y = rem(pos,n_y);
end
pos_x = (pos-pos_y)/n_y+1;
[pos_x pos_y]
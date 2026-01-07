pos=50;
if(rem(pos,n_u)==0)
    input = n_u;
else
    input = rem(pos,n_u);
end
pos = (pos-input)/n_u+1;
if(rem(pos,n_y)==0)
    pos_y = 5;
else
    pos_y = rem(pos,n_y);
end
pos_x = (pos-pos_y)/n_y+1;
[pos_x pos_y input]
function y = test(x, k)
	k = k;
	y = f2(x) +2;
end

function y2 = f2(x)
	global k;
	y2 = k*x;
end

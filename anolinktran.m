function t = anolinktran(a, b, c, d)

	if nargin == 4,
		alpha = a;
		an = b;
		theta = c;
		dn = d;
	else
		if numcols(a) < 4,
			error('too few columns in DH matrix');
		end
		alpha = a(1);
		an = a(2);
		if numcols(a) > 4,
			if a(5) == 0,	% revolute
				theta = b;
				dn = a(4);
			else		% prismatic
				theta = a(3);
				dn = b;
			end
		else
			theta = b;	% assume revolute if no sigma given
			dn = a(4);
		end
	end
	sa = sin(alpha); ca = cos(alpha);
	st = sin(theta); ct = cos(theta);
    
  t=   [   ct,      -st,        0,                     0;
          ca*st,   ca*ct,     -sa,               an*ca - dn*sa;
          sa*st,   sa*ct,      ca,               dn*ca + an*sa;
            0,       0,        0,                     1];
     
     
     
     
 
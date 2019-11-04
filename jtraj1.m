function [qt,qdt,qddt] = jtraj1(q0, q1, tv, qd0, qd1,qdd0,qdd1)
	if length(tv) > 1,
		tf = max(tv)
		t = tv(:);
	else
		tf= 1;
		t = [0:(tv-1)]'/(tv-1);	% normalized time from 0 -> 1
	end

	q0 = q0(:);
	q1 = q1(:);

	if nargin == 3,
		qd0 = zeros(size(q0));
		qd1 = qd0;
        qdd0 = qd0;
        qdd1 = qd0;
    elseif nargin>3
        qd0=qd0(:);
        qd1=qd1(:);
        qdd0=qdd0(:);
        qdd1=qdd1(:);
	end

	% compute the polynomial coefficients
	A = (12*(q1 - q0) - 6*(qd1+qd0)*tf-(qdd0-qdd1)*(tf^2))/2*(tf^3);
	B = (-30*(q1 - q0) + (16*qd0 + 14*qd1)*tf+(3*qdd0-2*qdd1)*(tf^2))/2*(tf^3);
	C = (20*(q1 - q0) - (12*qd0 + 8*qd1)*tf-(3*qdd0-qdd1)*(tf^2))/2*(tf^3);
    D = qdd0/2;
	E = qd0; % as the t vector has been normalized
	F = q0;

	tt = [t.^5 t.^4 t.^3 t.^2 t ones(size(t))];
	c = [A B C D E F]';
	
	qt = tt*c;

	% compute optional velocity
	if nargout >= 2,
		c = [ zeros(size(A)) 5*A 4*B 3*C  2*D E ]';
		qdt = tt*c;
	end

	% compute optional acceleration
	if nargout == 3,
		c = [ zeros(size(A))  zeros(size(A)) 20*A 12*B 6*C  2*D]';
		qddt = tt*c;
    end
 
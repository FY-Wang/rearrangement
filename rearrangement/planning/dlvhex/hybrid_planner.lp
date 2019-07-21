%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%% HYBRID PLANNING HEX PROGRAM %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% To test, disable external atoms then run: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% dlvhex test_problem.lp hybrid_planner.lp -n 1 -f moveTo %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Expected output: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% {moveTo(0,3,3),moveTo(1,4,0),moveTo(2,5,2),moveTo(3,6,4),moveTo(4,7,1), %%%%%%
% moveTo(5,8,5)} %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% location is inertial
location(B,C,T1) :- location(B,C,T), body(B), cell(C), time(T),
                    not -location(B,C,T1), T1=T+1,
                    T<ConstT, const(time_step,ConstT).

-location(B,C,T1) :- -location(B,C,T), body(B), cell(C), time(T),
                     not location(B,C,T1), T1=T+1, 
                     T<ConstT, const(time_step,ConstT).

% location is exogenous at T=0
location(B,C,0) v -location(B,C,0)  :- body(B), cell(C).
:- 2 <= #count { C : location(B,C,0) }, body(B).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% moveTo %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% moveTo is exogenous
moveTo(B,C,T) v -moveTo(B,C,T) :- body(B), cell(C), time(T),
                                  T<ConstT, const(time_step,ConstT).

% concurrency is also eliminated
:- 2 <= #count { B, C : moveTo(B,C,T)}, time(T),
   T<ConstT, const(time_step,ConstT).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% moveTo preconditions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% cannot move obstacles
:- moveTo(B,C,T), obs(B), cell(C), time(T),
   T<ConstT, const(time_step,ConstT).

% cannot moveTo a full cell
:- moveTo(B,C,T), body(B), location(B1,C,T), body(B1), B!=B1, cell(C), time(T), 
   T<ConstT, const(time_step,ConstT).

% cannot moveTo same location
:- moveTo(B,C,T), location(B,C,T), body(B), cell(C), time(T),
   T<ConstT, const(time_step,ConstT).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% moveTo effects %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
location(B,C,T1) :- moveTo(B,C,T), body(B), cell(C), time(T), T1=T+1,
                    T<ConstT, const(time_step,ConstT).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% moveTo ramifications %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
-location(B,C,T1) :- location(B,C,T), body(B), cell(C), time(T),
                     location(B,C1,T1), cell(C1), T1=T+1, C!=C1,
                     T<ConstT, const(time_step,ConstT).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% external atoms %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% cannot have collision at any timestep
:- &isCollision[location]().

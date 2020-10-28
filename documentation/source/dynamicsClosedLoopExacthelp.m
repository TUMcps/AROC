%% dynamicsClosedLoopExact
% system dynamics for the closed-loop system
% controlled with the exact control law
%
%% Syntax
%
% <html>
%        <div class="syntax">
% f = dynamicsClosedLoopExact(t,x,u,p,nx,dynamics) <br>
%        </div>
% </html>
%
%% Description
% This function implements the closed loop system dynamics for the
% case that the system is controlled by the nonlinear exact control
% law. Thereby, the original system dynamics of the benchmark models
% are extended by auxiliary states that store the linear part of the
% exact control law and that keep the control input constant during
% one time step. Further, the control input computed with the exact
% control law and insterted in the open-loop system dynamic of the
% benchmark modlel, which yields the equations for the closed-loop
% system.
%
%% Input Arguments
%
%
% <html>
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            x
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            system states x (dimension: [nx,1])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            u
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            disturbances = input to the closed loop controlled
%            system (dimension: [nx,1])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            p
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            parameter vector. Contains the optimal control inputs
%            for all vertices of the parallelotope
%            (dimension: [nu*(2^nx),1])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            nx
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of system states
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            dynamics
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            function handle to the function implementing the
%            open-loop dynamics of the system
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% Output Arguments
%
%
% <html>
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            f
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            value of the dynamic funcion = time derivative of x
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <convexInterpolationControlhelp.html |convexInterpolationControl|>, <matlab:doc('dynamcisClosedLoopLinear') dynamcisClosedLoopLinear>
%
%% References
% * *[1] Schuermann et al. (2017)*, Convex interpolation control with
% formal guarantees for disturbed and constrained nonlinear
% systems
%

%%
% <html>
%   <hr>
%   <p class="copy">&copy; 2018-2020 I6 Technische Universit&auml;t M&uuml;nchen
%        <tt class="minicdot">&#149;</tt>
%        <a href="https://tumcps.github.io/AROC/">Website</a>
%        <tt class="minicdot">&#149;</tt>
%        <a href="file:txts/LICENSE.txt">License</a>
%   </p>
% <div>
% <table>
%  <tr>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%             <img src="img/logoAroc.png" alt="logoAroc" height="40px">
%      </td>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%      <img src="img/logoCora.png" alt="logoCora" height="40px"></td>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%      <img src="img/logoChair.png" alt="logoChair" height="40px"></td>
%   <td style="background-color:#ffffff; border:0; width:25%; vertical-align:middle; text-align:center">
%      <img src="img/logoTum.png" alt="logoTum" height="40px"></td>
%  </tr>
% </table>
% </div>
% </html>

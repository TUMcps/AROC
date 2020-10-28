%% constraintfunctionFmincon
% constraints for the optimal control problems
% if solved with FMINCON
%
%% Syntax
%
% <html>
%        <div class="syntax">
% [c,ceq] = constraintfunctionFmincon(y, nx, nu, N, dt, x0, dynamics) <br>
%        </div>
% </html>
%
%% Description
% This function determines whether or not the constraints for the
% optimal control problems are satisfied, for the case that the
% optimal control problmes are solved with Matlab build-in function
% FMINCON
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
%            y
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            parameter vector for the optimization problem
%            (y = [x,u])
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
%            number of system states x
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            nu
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of control inputs u
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            N
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of time steps for the optimal control problem
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            dt
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            time step size for the optimal control problem
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            x0
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            initial state at the beginning of the optimal control
%            problem
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
%            function handle pointing to the function that
%            implements the system dynamics
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
%            c
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            amount of constraint violation for greater than
%            constraints
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            ceq
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            amount of constraint violation for equality
%            constraints
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <convexInterpolationControlhelp.html |convexInterpolationControl|>, <optimalControlFminconhelp.html |optimalControlFmincon|>
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

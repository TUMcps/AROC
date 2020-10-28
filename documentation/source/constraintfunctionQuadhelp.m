%% constraintfunctionQuad
% constraints for the optimization problem to
% determine the parameter of the quadratic control
% law
%
%% Syntax
%
% <html>
%        <div class="syntax">
% [c, ceq] = constraintfunctionQuad(y, nx, uMax, uMin) <br>
%        </div>
% </html>
%
%% Description
% This function determines whether or not the quadratic control law
% satisfies the input constraints. The optimization problem to
% determine the quadratic control law is solved with the  Matlab
% build-in function FMINCON
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
%            Parameter vector for the optimization problem
%            (dimension: [nu*(2*nx+1),1])
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
%            Number of system states x
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            uMax
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            vector containing the upper bound of the input
%            constraints
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            uMin
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            vector containing the lower bound of the input
%            constraints
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

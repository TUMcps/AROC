%% checkConstraint
% check if the solution to the optimal control problems
% satisfies all constraints
%
%% Syntax
%
% <html>
%        <div class="syntax">
% [res,L_,J_] = checkConstraint(x,u,L,J,Opts) <br>
%        </div>
% </html>
%
%% Description
% This function checks if the solution that was determined with the
% optimal control problems satisfies all constraints. This is
% necessary because due to the limited computation time the optimal
% control problems might be terminated before totoal convergence is
% achieved. Further, the function checks if the cost for the new
% solution is lower than the cost for the previous solution
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
%            Center trajectory determined by the optimal control problem
%            (dimension: [nx,N])
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
%            Control inputs determined by the optimal control problem
%            (dimension: [nu,N])
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            L
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            Value of the objective function (=cost) of the previous
%            solution
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            J
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            Summed distance of the points from the previous solution to
%            the terminal region
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            Opts
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            a structure containing following options
%        </p>
%        <p class="table">
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="inner">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .dT
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            time step. Prediction horizon: Opts.N * Opts.dT
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .Q
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            state weighting matrix for the cost function
%            (center trajectory)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .R
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            input weighting matrix for the cost function
%            (center trajectory)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .termReg
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            terminal region around the steady state xf
%            (class: mptPolytope)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .alpha
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            contraction rate for the contraction constraint
%            [&#123;0.1&#125; / alpha > 0]
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
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
%            res
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            Flag that specifies if all constraints are satisfied and
%            the cost of the new solution is lower than the one for the
%            previous solution (0 or 1)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            L_
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            Value of the objective function (=cost) of the new solution
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            J_
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            Summed distances of the points from the new solution to the
%            terminal region
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <reachsetMPChelp.html |reachsetMPC|>
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

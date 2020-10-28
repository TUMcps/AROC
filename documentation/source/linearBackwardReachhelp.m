%% linearBackwardReach
% backward reachable set for linearized system
%
%% Syntax
%
% <html>
%        <div class="syntax">
% [R,H] = linearBackwardReach(goalSet,xCenter,uCenter,U,Opts) <br>
%        </div>
% </html>
%
%% Description
% This function computes the backward reachable set starting from the
% goal set  for the inearized system.
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
%            goalSet
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            goal set that should be reached (class: zonotope)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            xCenter
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            cell-array containing the reference trajectory
%            states for all intermediate time steps
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            uCenter
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            cell-array containing the reference trajectory
%            inputs for all intermediate time steps
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            U
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            cell-array containing the sets of admissble control
%            inputs for all intermediate time steps
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
%            structure containing the following options
%        </p>
%        <p class="table">
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="inner">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .Ninter
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of intermediate time steps
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .nu
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of inputs
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .nw
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of disturbances
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .dt
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            time step size for one intermediate time step
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .funHandle
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            function handle for the dynamic function
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .linDyn.A
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            function handle to the system matrix of the
%            linearize dynamics
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .linDyn.B
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            function handle to the input matrix of the
%            linearize dynamics
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
%            R
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            backward reachable set (class: zonotope)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            H
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            matrix storing the generator-to-input assignment
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <safetyNetControlhelp.html |safetyNetControl|>
%
%% References
% * *[1] Schuermann et al. (2019)*, Formal Safety Net Control Using
% Backward Reachability Analysis
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

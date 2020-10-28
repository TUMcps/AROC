%% linearizeSystem
% compute linearized discrete time systems
%
%% Syntax
%
% <html>
%        <div class="syntax">
% [A,B,c,xf] = linearizeSystem(xCenter,uCenter,Opts) <br>
%        </div>
% </html>
%
%% Description
% This function computes the linearized discrete time system for all
% time steps along the reference trajectory
%
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
%             xCenter
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            reference trajectory states
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%             uCenter
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            reference trajectory inputs
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%             Opts
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            a struct containing the following fields
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
%             A
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            cell-array storing the system matrices of the linearized
%            time discrete system x(k+1) = A x(k) + B u(k) + c
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%             B
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            cell-array storing the input matrices of the linearized
%            time discrete system x(k+1) = A x(k) + B u(k) + c
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%             c
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            cell-array storing the constant offset of the linearized
%            time discrete system x(k+1) = A x(k) + B u(k) + c
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%             xf
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            cell-array storing the goal states for each time step
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <generatorSpaceControlhelp.html |generatorSpaceControl|>
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

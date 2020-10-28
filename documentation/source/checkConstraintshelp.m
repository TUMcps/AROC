%% checkConstraints
% check if the input and state constraints are satisfied
%
%% Syntax
%
% <html>
%        <div class="syntax">
% c = checkConstraints(R,K,u_ref,Opts) <br>
%        </div>
% </html>
%
%% Description
% This function checks if the reachable set satisfies the state and
% input constraints. The function returns a vector that specifies how
% much each constraint is violate, where c < 0 means that all
% constraints are satisfied.
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
%            R
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            cell-array storing the reachable set
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            K
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            feedback matrix for the control law u = u_ref + K(x-x_ref)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            u_ref
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            reference input for the control law u = u_ref + K(x-x_ref)
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
%            .stateCon
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            structure containing the state constraints
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .inputCon
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            structure containing the input constraints
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .nx
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            number of states
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
%            c
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            amount of violation for the inequality constraints
%            (c <= 0: no violation)
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <optimizationBasedControlhelp.html |optimizationBasedControl|>, <conFunhelp.html |conFun|>
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

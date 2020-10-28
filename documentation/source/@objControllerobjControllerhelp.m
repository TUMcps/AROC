%% objController
% abstract class for all controllers in AROC
%
%% Syntax
%
% <html>
%        <div class="syntax">
% obj = objController(dyn,Rfin,Param) <br>
% obj = objController(dyn,Rfin,Param,occSet) <br>
%        </div>
% </html>
%
%% Description
% This class is the superclass for all controller objects in the AROC
% toolbox. The class defines some common properties that are
% inherited by all controller objects.
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
%            dyn
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            function handle to the dynamic function of the
%            open-loop system
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            Rfin
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            final reachable set
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            Param
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            a structure containing the benchmark parameters
%        </p>
%        <p class="table">
% <table cellspacing="0" cellpadding="4" width="" border="1" frame="box" rules="none" class="inner">
% <tbody valign="top">
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .R0
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            initial set of states (class: interval)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .xf
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            goal state
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .tFinal
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            final time after which the goal state should be
%            reached
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .U
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            set of admissible control inputs (class:;
%            interval)
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            .W
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            set of uncertain disturbances (class: interval)
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            occSet
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            cell-array storing the occupancy set
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
%            obj
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            generated object of class objController
%        </p>
%    </td>
%    </tr>
% </tbody>
% </table>
% </html>
%
%% See Also
% <objConvInterContrhelp.html |objConvInterContr|>, <matlab:doc('objSafetyNetContr') objSafetyNetContr>, <objGenSpaceContrhelp.html |objGenSpaceContr|>
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

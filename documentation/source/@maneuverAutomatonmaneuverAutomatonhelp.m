%% maneuverAutomaton
% class representing a Maneuver Automaton
%
%% Syntax
%
% <html>
%        <div class="syntax">
% obj = maneuverAutomaton(primitives,shiftFun,shiftOccFun) <br>
%        </div>
% </html>
%
%% Description
% This class represents a Maneuver Automaton. The maneuver automaton
% is constructed from multiple motion primives computed with one of
% the control algorithms in the AROC toolbox. Afterwards, the
% automaton can be used to solve online control problems
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
%            primitives
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            cell-array storing the motion primitives. Each
%            motion primitive is an object of class
%            "objController"
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            shiftFun
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            function handle to a function that shifts the
%            initial set of a motion primitive to the final
%            state of the previous primitive
%        </p>
%    </td>
%    </tr>
%    <tr bgcolor="#F2F2F2">
%    <td>
%        <p class="table">
%            shiftOccFun
%        </p>
%    </td>
%    <td>
%        <p class="table">
%            function handle to a function that shifts the
%            occupancy set of a motion primitive to the final
%            state of the previous primitive
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
%            generated object of class maneuverAutomaton
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

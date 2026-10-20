<#
.SYNOPSIS
  Apply or revert the nRF54L15 Clean Core encrypted-ATT-ACK follow-up patch.

.DESCRIPTION
  Target:
    ...\nrf54l15_hal_parts\nrf54l15_hal_ble_peripheral_event_tx.inc

  The patch lets an encrypted ATT notification/indication catch the peer's
  ACK in the same connection event, so the next HID report can be sent in the
  next event instead of waiting to learn the ACK from the next event's
  central packet.

  The original file is backed up once as <target>.orig. -Revert restores it.
  This is a local experiment against core 1.0.22; a core update will overwrite
  the patch (and may change the anchors).

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\tools\patch-nrf54l15-encrypted-ack.ps1 -Apply

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\tools\patch-nrf54l15-encrypted-ack.ps1 -Revert
#>
[CmdletBinding()]
param(
  [switch]$Apply,
  [switch]$Revert,
  [string]$CorePath = ""
)

$ErrorActionPreference = "Stop"

if ($Apply -and $Revert) {
  throw "Use either -Apply or -Revert, not both."
}

if (-not $CorePath) {
  $pkg = Join-Path $env:LOCALAPPDATA "Arduino15\packages\nrf54l15clean\hardware\nrf54l15clean"
  if (-not (Test-Path -LiteralPath $pkg)) {
    throw "nRF54L15 core not found under $pkg. Pass -CorePath explicitly."
  }
  $versions = Get-ChildItem -LiteralPath $pkg -Directory |
      Where-Object { $_.Name -match '^\d+\.\d+\.\d+$' } |
      Sort-Object { [version]$_.Name } -Descending
  if (-not $versions) {
    throw "No versioned core directory found under $pkg."
  }
  $CorePath = $versions[0].FullName
}

$target = Join-Path $CorePath "libraries\Nrf54L15-Clean-Implementation\src\nrf54l15_hal_parts\nrf54l15_hal_ble_peripheral_event_tx.inc"
$backup = "$target.orig"

if (-not (Test-Path -LiteralPath $target)) {
  throw "Patch target not found: $target"
}

if ($Revert) {
  if (-not (Test-Path -LiteralPath $backup)) {
    throw "No backup to restore at $backup"
  }
  Copy-Item -LiteralPath $backup -Destination $target -Force
  Write-Host "Reverted: $target"
  exit 0
}

$content = [System.IO.File]::ReadAllText($target)
if ($content.Contains("allowEncryptedAckFollowup")) {
  Write-Host "Already patched: $target"
  exit 0
}

if (-not (Test-Path -LiteralPath $backup)) {
  Copy-Item -LiteralPath $target -Destination $backup
  Write-Host "Backup: $backup"
}

$nl = if ($content.Contains("`r`n")) { "`r`n" } else { "`n" }
function Normalize-Newlines([string]$text, [string]$newline) {
  return $text.Replace("`r`n", "`n").Replace("`n", $newline)
}

$pairs = @(
  @{
    Old = @'
  const bool allowQueuedSmpFollowupTurnaround =
      !terminateInd &&
      !connectionEncSessionValid_ &&
      !connectionEncRxEnabled_ &&
      !connectionEncTxEnabled_ &&
      !connectionEncStartReqPending_ &&
      !connectionEncStartReqTxPending_ &&
      !connectionEncAwaitingStartRsp_ &&
      !connectionEncKeyDerivationPending_ &&
      txIsSmpPlain;
  const bool doPostTxRxTurnaround =
      !terminateInd &&
      (allowPlainFollowupTurnaround ||
       allowQueuedSmpFollowupTurnaround ||
       (connectionEncStartReqPending_ && txIsEncRspPlain) ||
       txIsStartEncReqPlain);
'@
    New = @'
  const bool allowQueuedSmpFollowupTurnaround =
      !terminateInd &&
      !connectionEncSessionValid_ &&
      !connectionEncRxEnabled_ &&
      !connectionEncTxEnabled_ &&
      !connectionEncStartReqPending_ &&
      !connectionEncStartReqTxPending_ &&
      !connectionEncAwaitingStartRsp_ &&
      !connectionEncKeyDerivationPending_ &&
      txIsSmpPlain;
  // Encrypted ATT value pushes (HID input notifications). Listen for the
  // peer's ACK in the same connection event so the next report can go out in
  // the next event, instead of waiting to learn the ACK from the next event's
  // central packet. Only empty ACKs are consumed here; encrypted data/control
  // PDUs are left to the normal RX path (see the guard before the decrypt
  // block below). The listen budget is shortened below; the 4 ms handshake
  // budget is far too long to pay after every report.
  const bool txIsAttValuePush =
      txPayloadIsNewPlain &&
      (txLlid == kBlePduDataStartOrComplete) &&
      (txLength >= (kBleL2capHeaderLen + 3U)) &&
      (readLe16(&connectionLastTxPlainPayload_[2]) == kBleL2capCidAtt) &&
      ((connectionLastTxPlainPayload_[4] == kAttOpHandleValueNtf) ||
       (connectionLastTxPlainPayload_[4] == kAttOpHandleValueInd));
  const bool allowEncryptedAckFollowup =
      !terminateInd &&
      connectionEncSessionValid_ &&
      connectionEncRxEnabled_ &&
      connectionEncTxEnabled_ &&
      !connectionEncStartReqPending_ &&
      !connectionEncStartReqTxPending_ &&
      !connectionEncAwaitingStartRsp_ &&
      !connectionEncKeyDerivationPending_ &&
      txIsAttValuePush;
  const bool doPostTxRxTurnaround =
      !terminateInd &&
      (allowPlainFollowupTurnaround ||
       allowQueuedSmpFollowupTurnaround ||
       allowEncryptedAckFollowup ||
       (connectionEncStartReqPending_ && txIsEncRspPlain) ||
       txIsStartEncReqPlain);
'@
  },
  @{
    Old = @'
    if (((connectionCurrentRxPhy_ & kBlePhyCoded) == 0U) &&
        followIntervalUs > 3000U) {
      const uint32_t capUs = followIntervalUs - 3000U;
      if (followListenUs > capUs) {
        followListenUs = capUs;
      }
    }
    encDebug_.connLastFollowupListenBudgetUs = followListenUs;
'@
    New = @'
    if (((connectionCurrentRxPhy_ & kBlePhyCoded) == 0U) &&
        followIntervalUs > 3000U) {
      const uint32_t capUs = followIntervalUs - 3000U;
      if (followListenUs > capUs) {
        followListenUs = capUs;
      }
    }
    if (allowEncryptedAckFollowup) {
      // One T_IFS plus an empty ACK is enough; the loop breaks as soon as the
      // ACK is processed. Coded PHY uses a 300 us T_IFS, so keep a larger
      // fallback there. This is only the timeout when the ACK does not arrive.
      const uint32_t encAckBudgetUs =
          ((connectionCurrentRxPhy_ & kBlePhyCoded) != 0U) ? 2500U : 800U;
      if (followListenUs > encAckBudgetUs) {
        followListenUs = encAckBudgetUs;
      }
    }
    encDebug_.connLastFollowupListenBudgetUs = followListenUs;
'@
  },
  @{
    Old = @'
        bool peerAckedLastTxFollow = sequenceFollow.peerAcknowledgedLastTx;
        bool packetIsNewFollow = sequenceFollow.packetIsNew;

        uint8_t rxLengthFollow = rxLengthFollowRaw;
        if (connectionEncSessionValid_ &&
'@
    New = @'
        bool peerAckedLastTxFollow = sequenceFollow.peerAcknowledgedLastTx;
        bool packetIsNewFollow = sequenceFollow.packetIsNew;

        const bool followIsEmptyAck =
            (llidFollow == kBlePduDataContinuation) &&
            (rxLengthFollowRaw == 0U);
        if (allowEncryptedAckFollowup && !followIsEmptyAck) {
          // This follow-up is encrypted data or control traffic. Leave it to
          // the normal RX path in the next connection event: do not decrypt
          // it or advance the receive counter here, so the central
          // retransmits it and the normal path can process it.
          break;
        }

        uint8_t rxLengthFollow = rxLengthFollowRaw;
        if (connectionEncSessionValid_ &&
'@
  },
  @{
    Old = @'
        if (!terminateInd) {
          if (peerAckedLastTxFollow) {
            noteLastSmpTxAcknowledged();
          }
          const BleLlAuthenticatedSequenceUpdate sequenceUpdate =
              bleLlApplyAuthenticatedSequence(
                  sequenceFollow, connectionExpectedRxSn_, connectionTxSn_,
                  connectionTxHistoryValid_);
          connectionExpectedRxSn_ = sequenceUpdate.expectedRxSn;
          connectionTxSn_ = sequenceUpdate.nextTxSn;
          connectionTxHistoryValid_ = sequenceUpdate.txHistoryValid;
          connectionFreshTxAllowed_ = sequenceUpdate.freshTxAllowed;
        } else {
          peerAckedLastTxFollow = false;
          packetIsNewFollow = false;
        }
'@
    New = @'
        if (!terminateInd) {
          if (peerAckedLastTxFollow) {
            noteLastSmpTxAcknowledged();
          }
          const BleLlAuthenticatedSequenceUpdate sequenceUpdate =
              bleLlApplyAuthenticatedSequence(
                  sequenceFollow, connectionExpectedRxSn_, connectionTxSn_,
                  connectionTxHistoryValid_);
          connectionExpectedRxSn_ = sequenceUpdate.expectedRxSn;
          connectionTxSn_ = sequenceUpdate.nextTxSn;
          connectionTxHistoryValid_ = sequenceUpdate.txHistoryValid;
          connectionFreshTxAllowed_ = sequenceUpdate.freshTxAllowed;
        } else {
          peerAckedLastTxFollow = false;
          packetIsNewFollow = false;
        }
        if (allowEncryptedAckFollowup && followIsEmptyAck) {
          // The peer acknowledged the notification. The next event can carry
          // the next report; no need to wait out the listen budget.
          break;
        }
'@
  }
)

foreach ($pair in $pairs) {
  $old = Normalize-Newlines $pair.Old $nl
  $new = Normalize-Newlines $pair.New $nl
  $first = $content.IndexOf($old)
  if ($first -lt 0) {
    throw "Patch anchor not found. The installed core differs from 1.0.22; edit the anchors in this script."
  }
  if ($content.IndexOf($old, $first + 1) -ge 0) {
    throw "Patch anchor is not unique. Refusing to patch."
  }
  $content = $content.Replace($old, $new)
}

[System.IO.File]::WriteAllText($target, $content, (New-Object System.Text.UTF8Encoding($false)))
Write-Host "Patched: $target"
Write-Host "Original: $backup"

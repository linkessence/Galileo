/** @file
  BDS Lib functions which relate with create or process the boot option.

Copyright (c) 2013 Intel Corporation.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in
the documentation and/or other materials provided with the
distribution.
* Neither the name of Intel Corporation nor the names of its
contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
--*/

#include "InternalBdsLib.h"

CONST UINT16 USB_LANG_ID   = 0x0409; // English
CHAR16       mUefiPrefix[] = L"UEFI ";

EFI_BOOT_MANAGER_REFRESH_LEGACY_BOOT_OPTION  mEfiBootManagerRefreshLegacyBootOption = NULL;
EFI_BOOT_MANAGER_LEGACY_BOOT                 mEfiBootManagerLegacyBoot              = NULL;

///
/// This GUID is used for an EFI Variable that stores the front device pathes
/// for a partial device path that starts with the HD node.
///
EFI_GUID mHdBootVariablePrivateGuid = { 0xfab7e9e1, 0x39dd, 0x4f2b, { 0x84, 0x08, 0xe2, 0x0e, 0x90, 0x6c, 0xb6, 0xde } };
EFI_GUID mAutoCreateBootOptionGuid  = { 0x8108ac4e, 0x9f11, 0x4d59, { 0x85, 0x0e, 0xe2, 0x1a, 0x52, 0x2c, 0x59, 0xb2 } };

/**
  Get the image file buffer data and buffer size by its device path. 

  @param FilePath  On input, a pointer to an allocated buffer containing the device
                   path of the file.
                   On output the pointer could be NULL when the function fails to
                   load the boot option, or could point to an allocated buffer containing
                   the device path of the file.
                   It could be updated by either short-form device path expanding,
                   or default boot file path appending.
                   Caller is responsible to free it when it's non-NULL.
  @param FileSize  A pointer to the size of the file buffer.

  @retval NULL   File is NULL, or FileSize is NULL. Or, the file can't be found.
  @retval other  The file buffer. The caller is responsible to free the memory.
**/
VOID *
LoadEfiBootOption (
  IN OUT EFI_DEVICE_PATH_PROTOCOL **FilePath,
  OUT    UINTN                    *FileSize
  );

VOID
EFIAPI
EfiBootManagerRegisterLegacyBootSupport (
  EFI_BOOT_MANAGER_REFRESH_LEGACY_BOOT_OPTION   RefreshLegacyBootOption,
  EFI_BOOT_MANAGER_LEGACY_BOOT                  LegacyBoot
  )
{
  mEfiBootManagerRefreshLegacyBootOption = RefreshLegacyBootOption;
  mEfiBootManagerLegacyBoot              = LegacyBoot;
}

VOID
FreeAndSet (
  VOID   **Orig,
  VOID   *New
  )
{
  FreePool (*Orig);
  *Orig = New;
}

/**
  Internal function to check if the input boot option is a valid EFI NV Boot####.

  @param OptionToCheck  Boot option to be checked.

  @retval TRUE      This boot option matches a valid EFI NV Boot####.
  @retval FALSE     If not.

**/
BOOLEAN
BootOptionInVariable (
  IN  EFI_BOOT_MANAGER_LOAD_OPTION             *OptionToCheck
  )
{
  EFI_STATUS                   Status;
  EFI_BOOT_MANAGER_LOAD_OPTION BootOption;
  BOOLEAN                      Valid;
  CHAR16                       OptionName[sizeof ("Boot####")];

  UnicodeSPrint (OptionName, sizeof (OptionName), L"Boot%04x", OptionToCheck->OptionNumber);
  Status = EfiBootManagerVariableToLoadOption (OptionName, &BootOption);
  if (EFI_ERROR (Status)) {
    return FALSE;
  }

  //
  // If the Boot Option Number and Device Path matches, OptionToCheck matches a
  // valid EFI NV Boot####.
  //
  Valid = FALSE;
  if ((OptionToCheck->OptionNumber == BootOption.OptionNumber) &&
      (CompareMem (OptionToCheck->FilePath, BootOption.FilePath, GetDevicePathSize (OptionToCheck->FilePath)) == 0))
      {
    Valid = TRUE;
  }

  EfiBootManagerFreeLoadOption (&BootOption);
  return Valid;
}

/**
  According to a file guild, check a Fv file device path is valid. If it is invalid,
  try to return the valid device path.
  FV address maybe changes for memory layout adjust from time to time, use this function
  could promise the Fv file device path is right.

  @param  DevicePath   The Fv file device path to be fixed up.

  @retval DevicePath   The DevicePath ponits to the valid FV file.
  @retval !NULL        The fixed device path.
  @retval NULL         Failed to fix up the DevicePath.
**/
VOID
FixupMemmapFvFilePath (
  IN OUT EFI_DEVICE_PATH_PROTOCOL      **DevicePath
  )
{
  EFI_STATUS                    Status;
  UINTN                         Index;
  EFI_DEVICE_PATH_PROTOCOL      *Node;
  EFI_HANDLE                    FvHandle;
  EFI_FIRMWARE_VOLUME2_PROTOCOL *Fv;
  EFI_LOADED_IMAGE_PROTOCOL     *LoadedImage;
  UINTN                         Size;
  EFI_FV_FILETYPE               Type;
  EFI_FV_FILE_ATTRIBUTES        Attributes;
  UINT32                        AuthenticationStatus;
  UINTN                         FvHandleCount;
  EFI_HANDLE                    *FvHandleBuffer;
  EFI_DEVICE_PATH_PROTOCOL      *NewDevicePath;
  
  Node = *DevicePath;
  Status = gBS->LocateDevicePath (&gEfiFirmwareVolume2ProtocolGuid, &Node, &FvHandle);
  if (!EFI_ERROR (Status)) {
    Status = gBS->HandleProtocol (FvHandle, &gEfiFirmwareVolume2ProtocolGuid, (VOID **) &Fv);
    ASSERT_EFI_ERROR (Status);

    Status = Fv->ReadFile (
                   Fv,
                   EfiGetNameGuidFromFwVolDevicePathNode ((CONST MEDIA_FW_VOL_FILEPATH_DEVICE_PATH *) Node),
                   NULL,
                   &Size,
                   &Type,
                   &Attributes,
                   &AuthenticationStatus
                   );
    if (EFI_ERROR (Status)) {
      FreeAndSet ((VOID **) DevicePath, NULL);
    }
    return;
  }

    
  Node = NextDevicePathNode (DevicePath);

  //
  // Firstly find the FV file in current FV
  //
  gBS->HandleProtocol (
         gImageHandle,
         &gEfiLoadedImageProtocolGuid,
         (VOID **) &LoadedImage
         );
  NewDevicePath = AppendDevicePathNode (DevicePathFromHandle (LoadedImage->DeviceHandle), Node);
  FixupMemmapFvFilePath (&NewDevicePath);

  if (NewDevicePath != NULL) {
    FreeAndSet ((VOID **) DevicePath, NewDevicePath);
    return;
  }

  //
  // Secondly find the FV file in all other FVs
  //
  gBS->LocateHandleBuffer (
         ByProtocol,
         &gEfiFirmwareVolume2ProtocolGuid,
         NULL,
         &FvHandleCount,
         &FvHandleBuffer
         );
  for (Index = 0; Index < FvHandleCount; Index++) {
    if (FvHandleBuffer[Index] == LoadedImage->DeviceHandle) {
      //
      // Skip current FV
      //
      continue;
    }
    NewDevicePath = AppendDevicePathNode (DevicePathFromHandle (FvHandleBuffer[Index]), Node);
    FixupMemmapFvFilePath (&NewDevicePath);

    if (NewDevicePath != NULL) {
      FreeAndSet ((VOID **) DevicePath, NewDevicePath);
      return;
    }
  }
}

/**
  Check if it's of Fv file device path type.
  
  The function doesn't garentee the device path points to existing Fv file.

  @retval TRUE   The device path is of Fv file device path type.
  @retval FALSE  The device path isn't of Fv file device path type.
**/
BOOLEAN
IsMemmapFvFilePath (
  IN EFI_DEVICE_PATH_PROTOCOL    *DevicePath
  )
{
  EFI_DEVICE_PATH_PROTOCOL   *FileNode;

  if ((DevicePathType (DevicePath) == HARDWARE_DEVICE_PATH) && (DevicePathSubType (DevicePath) == HW_MEMMAP_DP)) {
    FileNode = NextDevicePathNode (DevicePath);
    if ((DevicePathType (FileNode) == MEDIA_DEVICE_PATH) && (DevicePathSubType (FileNode) == MEDIA_PIWG_FW_FILE_DP)) {
      return IsDevicePathEnd (NextDevicePathNode (FileNode));
    }
  }

  return FALSE;
}

/**
  Check whether a USB device match the specified USB Class device path. This
  function follows "Load Option Processing" behavior in UEFI specification.

  @param UsbIo       USB I/O protocol associated with the USB device.
  @param UsbClass    The USB Class device path to match.

  @retval TRUE       The USB device match the USB Class device path.
  @retval FALSE      The USB device does not match the USB Class device path.

**/
BOOLEAN
MatchUsbClass (
  IN EFI_USB_IO_PROTOCOL        *UsbIo,
  IN USB_CLASS_DEVICE_PATH      *UsbClass
  )
{
  EFI_STATUS                    Status;
  EFI_USB_DEVICE_DESCRIPTOR     DevDesc;
  EFI_USB_INTERFACE_DESCRIPTOR  IfDesc;
  UINT8                         DeviceClass;
  UINT8                         DeviceSubClass;
  UINT8                         DeviceProtocol;

  if ((DevicePathType (UsbClass) != MESSAGING_DEVICE_PATH) ||
      (DevicePathSubType (UsbClass) != MSG_USB_CLASS_DP)){
    return FALSE;
  }

  //
  // Check Vendor Id and Product Id.
  //
  Status = UsbIo->UsbGetDeviceDescriptor (UsbIo, &DevDesc);
  if (EFI_ERROR (Status)) {
    return FALSE;
  }

  if ((UsbClass->VendorId != 0xffff) &&
      (UsbClass->VendorId != DevDesc.IdVendor)) {
    return FALSE;
  }

  if ((UsbClass->ProductId != 0xffff) &&
      (UsbClass->ProductId != DevDesc.IdProduct)) {
    return FALSE;
  }

  DeviceClass    = DevDesc.DeviceClass;
  DeviceSubClass = DevDesc.DeviceSubClass;
  DeviceProtocol = DevDesc.DeviceProtocol;
  if (DeviceClass == 0) {
    //
    // If Class in Device Descriptor is set to 0, use the Class, SubClass and
    // Protocol in Interface Descriptor instead.
    //
    Status = UsbIo->UsbGetInterfaceDescriptor (UsbIo, &IfDesc);
    if (EFI_ERROR (Status)) {
      return FALSE;
    }

    DeviceClass    = IfDesc.InterfaceClass;
    DeviceSubClass = IfDesc.InterfaceSubClass;
    DeviceProtocol = IfDesc.InterfaceProtocol;
  }

  //
  // Check Class, SubClass and Protocol.
  //
  if ((UsbClass->DeviceClass != 0xff) &&
      (UsbClass->DeviceClass != DeviceClass)) {
    return FALSE;
  }

  if ((UsbClass->DeviceSubClass != 0xff) &&
      (UsbClass->DeviceSubClass != DeviceSubClass)) {
    return FALSE;
  }

  if ((UsbClass->DeviceProtocol != 0xff) &&
      (UsbClass->DeviceProtocol != DeviceProtocol)) {
    return FALSE;
  }

  return TRUE;
}

/**
  Eliminate the extra spaces in the Str to one space.
**/
VOID
EliminateExtraSpaces (
  IN CHAR16                    *Str
  )
{
  UINTN                        Index;
  UINTN                        ActualIndex;

  for (Index = 0, ActualIndex = 0; Str[Index] != L'\0'; Index++) {
    if ((Str[Index] != L' ') || ((ActualIndex > 0) && (Str[ActualIndex - 1] != L' '))) {
      Str[ActualIndex++] = Str[Index];
    }
  }
  Str[ActualIndex] = L'\0';
}

/**
  Try to get the controller's ATA/ATAPI description.

  @param Handle                Controller handle.

  @return  The description string.
**/
CHAR16 *
GetAtaAtapiDescription (
  IN EFI_HANDLE                Handle
  )
{
  UINTN                        Index;
  EFI_STATUS                   Status;
  EFI_DISK_INFO_PROTOCOL       *DiskInfo;
  UINT32                       BufferSize;
  EFI_ATAPI_IDENTIFY_DATA      IdentifyData;
  CHAR16                       *Description;
  UINTN                        Length;
  CONST UINTN                  ModelNameLength    = 40;
  CONST UINTN                  SerialNumberLength = 20;

  Status = gBS->HandleProtocol (
                  Handle,
                  &gEfiDiskInfoProtocolGuid,
                  (VOID **) &DiskInfo
                  );
  if (EFI_ERROR (Status)) {
    return NULL;
  }

  Description  = NULL;
  BufferSize   = sizeof (EFI_ATAPI_IDENTIFY_DATA);
  Status = DiskInfo->Identify (
                       DiskInfo,
                       &IdentifyData,
                       &BufferSize
                       );
  if (!EFI_ERROR (Status)) {
    Description = AllocateZeroPool ((ModelNameLength + SerialNumberLength + 2) * sizeof (CHAR16));
    ASSERT (Description != NULL);
    for (Index = 0; Index + 1 < ModelNameLength; Index += 2) {
      Description[Index]     = (CHAR16) IdentifyData.ModelName[Index + 1];
      Description[Index + 1] = (CHAR16) IdentifyData.ModelName[Index];
    }

    Length = Index;
    Description[Length++] = L' ';

    for (Index = 0; Index + 1 < SerialNumberLength; Index += 2) {
      Description[Length + Index]     = (CHAR16) IdentifyData.SerialNo[Index + 1];
      Description[Length + Index + 1] = (CHAR16) IdentifyData.SerialNo[Index];
    }
    Length += Index;
    Description[Length++] = L'\0';
    ASSERT (Length == ModelNameLength + SerialNumberLength + 2);

    EliminateExtraSpaces (Description);
  }

  return Description;
}

/**
  Try to get the controller's USB description.

  @param Handle                Controller handle.

  @return  The description string.
**/
CHAR16 *
GetUsbDescription (
  IN EFI_HANDLE                Handle
  )
{
  EFI_STATUS                   Status;
  EFI_USB_IO_PROTOCOL          *UsbIo;
  CHAR16                       NullChar;
  CHAR16                       *Manufacturer;
  CHAR16                       *Product;
  CHAR16                       *SerialNumber;
  CHAR16                       *Description;
  EFI_USB_DEVICE_DESCRIPTOR    DevDesc;

  Status = gBS->HandleProtocol (
                  Handle,
                  &gEfiUsbIoProtocolGuid,
                  (VOID **) &UsbIo
                  );
  if (EFI_ERROR (Status)) {
    return NULL;
  }

  NullChar = L'\0';

  Status = UsbIo->UsbGetDeviceDescriptor (UsbIo, &DevDesc);
  if (EFI_ERROR (Status)) {
    return NULL;
  }

  Status = UsbIo->UsbGetStringDescriptor (
                    UsbIo,
                    USB_LANG_ID,
                    DevDesc.StrManufacturer,
                    &Manufacturer
                    );
  if (EFI_ERROR (Status)) {
    Manufacturer = &NullChar;
  }
  
  Status = UsbIo->UsbGetStringDescriptor (
                    UsbIo,
                    USB_LANG_ID,
                    DevDesc.StrProduct,
                    &Product
                    );
  if (EFI_ERROR (Status)) {
    Product = &NullChar;
  }
  
  Status = UsbIo->UsbGetStringDescriptor (
                    UsbIo,
                    USB_LANG_ID,
                    DevDesc.StrSerialNumber,
                    &SerialNumber
                    );
  if (EFI_ERROR (Status)) {
    SerialNumber = &NullChar;
  }

  if ((Manufacturer == &NullChar) &&
      (Product == &NullChar) &&
      (SerialNumber == &NullChar)
      ) {
    return NULL;
  }

  Description = AllocateZeroPool (StrSize (Manufacturer) + StrSize (Product) + StrSize (SerialNumber));
  ASSERT (Description != NULL);
  StrCat (Description, Manufacturer);
  StrCat (Description, L" ");

  StrCat (Description, Product);  
  StrCat (Description, L" ");

  StrCat (Description, SerialNumber);

  if (Manufacturer != &NullChar) {
    FreePool (Manufacturer);
  }
  if (Product != &NullChar) {
    FreePool (Product);
  }
  if (SerialNumber != &NullChar) {
    FreePool (SerialNumber);
  }

  EliminateExtraSpaces (Description);

  return Description;
}

/**
  Return the boot description for the controller based on the type.

  @param Handle                Controller handle.

  @return  The description string.
**/
CHAR16 *
GetMiscDescription (
  IN EFI_HANDLE                  Handle
  )
{
  EFI_STATUS                     Status;
  CHAR16                         *Description;
  EFI_BLOCK_IO_PROTOCOL          *BlockIo;

  switch (BootTypeFromDevicePath (DevicePathFromHandle (Handle))) {
  case AcpiFloppyBoot:
    Description = L"Floppy";
    break;

  case MessageAtapiBoot:
  case MessageSataBoot:
    Status = gBS->HandleProtocol (Handle, &gEfiBlockIoProtocolGuid, (VOID **) &BlockIo);
    ASSERT_EFI_ERROR (Status);
    //
    // Assume a removable SATA device should be the DVD/CD device
    //
    Description = BlockIo->Media->RemovableMedia ? L"DVD/CDROM" : L"Hard Drive";
    break;

  case MessageUsbBoot:
    Description = L"USB Device";
    break;

  case MessageScsiBoot:
    Description = L"SCSI Device";
    break;

  default:
    Description = L"Misc Device";
    break;
  }

  return AllocateCopyPool (StrSize (Description), Description);
}

GET_BOOT_DESCRIPTION mGetBootDescription[] = {
  GetUsbDescription,
  GetAtaAtapiDescription,
  GetMiscDescription
};

/**
  Check whether a USB device match the specified USB WWID device path. This
  function follows "Load Option Processing" behavior in UEFI specification.

  @param UsbIo       USB I/O protocol associated with the USB device.
  @param UsbWwid     The USB WWID device path to match.

  @retval TRUE       The USB device match the USB WWID device path.
  @retval FALSE      The USB device does not match the USB WWID device path.

**/
BOOLEAN
MatchUsbWwid (
  IN EFI_USB_IO_PROTOCOL        *UsbIo,
  IN USB_WWID_DEVICE_PATH       *UsbWwid
  )
{
  EFI_STATUS                   Status;
  EFI_USB_DEVICE_DESCRIPTOR    DevDesc;
  EFI_USB_INTERFACE_DESCRIPTOR IfDesc;
  UINT16                       *LangIdTable;
  UINT16                       TableSize;
  UINT16                       Index;
  CHAR16                       *CompareStr;
  UINTN                        CompareLen;
  CHAR16                       *SerialNumberStr;
  UINTN                        Length;

  if ((DevicePathType (UsbWwid) != MESSAGING_DEVICE_PATH) ||
      (DevicePathSubType (UsbWwid) != MSG_USB_WWID_DP)) {
    return FALSE;
  }

  //
  // Check Vendor Id and Product Id.
  //
  Status = UsbIo->UsbGetDeviceDescriptor (UsbIo, &DevDesc);
  if (EFI_ERROR (Status)) {
    return FALSE;
  }
  if ((DevDesc.IdVendor != UsbWwid->VendorId) ||
      (DevDesc.IdProduct != UsbWwid->ProductId)) {
    return FALSE;
  }

  //
  // Check Interface Number.
  //
  Status = UsbIo->UsbGetInterfaceDescriptor (UsbIo, &IfDesc);
  if (EFI_ERROR (Status)) {
    return FALSE;
  }
  if (IfDesc.InterfaceNumber != UsbWwid->InterfaceNumber) {
    return FALSE;
  }

  //
  // Check Serial Number.
  //
  if (DevDesc.StrSerialNumber == 0) {
    return FALSE;
  }

  //
  // Get all supported languages.
  //
  TableSize = 0;
  LangIdTable = NULL;
  Status = UsbIo->UsbGetSupportedLanguages (UsbIo, &LangIdTable, &TableSize);
  if (EFI_ERROR (Status) || (TableSize == 0) || (LangIdTable == NULL)) {
    return FALSE;
  }

  //
  // Serial number in USB WWID device path is the last 64-or-less UTF-16 characters.
  //
  CompareStr = (CHAR16 *) (UINTN) (UsbWwid + 1);
  CompareLen = (DevicePathNodeLength (UsbWwid) - sizeof (USB_WWID_DEVICE_PATH)) / sizeof (CHAR16);
  if (CompareStr[CompareLen - 1] == L'\0') {
    CompareLen--;
  }

  //
  // Compare serial number in each supported language.
  //
  for (Index = 0; Index < TableSize / sizeof (UINT16); Index++) {
    SerialNumberStr = NULL;
    Status = UsbIo->UsbGetStringDescriptor (
                      UsbIo,
                      LangIdTable[Index],
                      DevDesc.StrSerialNumber,
                      &SerialNumberStr
                      );
    if (EFI_ERROR (Status) || (SerialNumberStr == NULL)) {
      continue;
    }

    Length = StrLen (SerialNumberStr);
    if ((Length >= CompareLen) &&
        (CompareMem (SerialNumberStr + Length - CompareLen, CompareStr, CompareLen * sizeof (CHAR16)) == 0)) {
      FreePool (SerialNumberStr);
      return TRUE;
    }

    FreePool (SerialNumberStr);
  }

  return FALSE;
}

VOID
PrintDp (
  EFI_DEVICE_PATH_PROTOCOL            *DevicePath
  )
{
  EFI_STATUS                          Status;
  EFI_DEVICE_PATH_TO_TEXT_PROTOCOL    *ToText;
  CHAR16                              *DpStr;

  Status = gBS->LocateProtocol (&gEfiDevicePathToTextProtocolGuid, NULL, (VOID **) &ToText);
  ASSERT_EFI_ERROR (Status);

  DpStr = ToText->ConvertDevicePathToText (DevicePath, FALSE, FALSE);
  DEBUG ((EFI_D_INFO, "%s", DpStr));
  FreePool (DpStr);
}

/**
  Find a USB device which match the specified short-form device path start with 
  USB Class or USB WWID device path. If ParentDevicePath is NULL, this function
  will search in all USB devices of the platform. If ParentDevicePath is not NULL,
  this function will only search in its child devices.

  @param DevicePath           The device path that contains USB Class or USB WWID device path.
  @param ParentDevicePathSize The length of the device path before the USB Class or 
                              USB WWID device path.
  @param UsbIoHandleCount     A pointer to the count of the returned USB IO handles.

  @retval NULL       The matched USB IO handles cannot be found.
  @retval other      The matched USB IO handles.

**/
EFI_HANDLE *
FindUsbDevice (
  IN  EFI_DEVICE_PATH_PROTOCOL  *DevicePath,
  IN  UINTN                     ParentDevicePathSize,
  OUT UINTN                     *UsbIoHandleCount
  )
{
  EFI_STATUS                Status;
  EFI_HANDLE                *UsbIoHandles;
  EFI_DEVICE_PATH_PROTOCOL  *UsbIoDevicePath;
  EFI_USB_IO_PROTOCOL       *UsbIo;
  UINTN                     Index;
  UINTN                     UsbIoDevicePathSize;
  BOOLEAN                   Matched;

  ASSERT (UsbIoHandleCount != NULL);  

  //
  // Get all UsbIo Handles.
  //
  Status = gBS->LocateHandleBuffer (
                  ByProtocol,
                  &gEfiUsbIoProtocolGuid,
                  NULL,
                  UsbIoHandleCount,
                  &UsbIoHandles
                  );
  if (EFI_ERROR (Status)) {
    *UsbIoHandleCount = 0;
    UsbIoHandles      = NULL;
  }

  for (Index = 0; Index < *UsbIoHandleCount; ) {
    //
    // Get the Usb IO interface.
    //
    Status = gBS->HandleProtocol(
                    UsbIoHandles[Index],
                    &gEfiUsbIoProtocolGuid,
                    (VOID **) &UsbIo
                    );
    UsbIoDevicePath = DevicePathFromHandle (UsbIoHandles[Index]);
    Matched         = FALSE;
    if (!EFI_ERROR (Status) && (UsbIoDevicePath != NULL)) {
      UsbIoDevicePathSize = GetDevicePathSize (UsbIoDevicePath) - END_DEVICE_PATH_LENGTH;

      //
      // Compare starting part of UsbIoHandle's device path with ParentDevicePath.
      //
      if (CompareMem (UsbIoDevicePath, DevicePath, ParentDevicePathSize) == 0) {
        if (MatchUsbClass (UsbIo, (USB_CLASS_DEVICE_PATH *) ((UINTN) DevicePath + ParentDevicePathSize)) ||
            MatchUsbWwid (UsbIo, (USB_WWID_DEVICE_PATH *) ((UINTN) DevicePath + ParentDevicePathSize))) {
          Matched = TRUE;
        }
      }
    }

    if (!Matched) {
      (*UsbIoHandleCount) --;
      CopyMem (&UsbIoHandles[Index], &UsbIoHandles[Index + 1], (*UsbIoHandleCount - Index) * sizeof (EFI_HANDLE));
    } else {
      Index++;
    }
  }

  return UsbIoHandles;
}

/**
  Expand USB Class or USB WWID device path node to be full device path of a USB
  device in platform.

  This function support following 4 cases:
  1) Boot Option device path starts with a USB Class or USB WWID device path,
     and there is no Media FilePath device path in the end.
     In this case, it will follow Removable Media Boot Behavior.
  2) Boot Option device path starts with a USB Class or USB WWID device path,
     and ended with Media FilePath device path.
  3) Boot Option device path starts with a full device path to a USB Host Controller,
     contains a USB Class or USB WWID device path node, while not ended with Media
     FilePath device path. In this case, it will follow Removable Media Boot Behavior.
  4) Boot Option device path starts with a full device path to a USB Host Controller,
     contains a USB Class or USB WWID device path node, and ended with Media
     FilePath device path.

  @param  DevicePath    On input, a pointer to an allocated buffer that contains the 
                        file device path.
                        On output, a pointer to an reallocated buffer that contains 
                        the expanded device path. It would point to NULL if the file
                        cannot be read.

  @param  FileSize      A pointer to the file size.

  @retval !NULL  The file buffer.
  @retval NULL   The input device path doesn't point to a valid file.
**/
VOID *
ExpandUsbShortFormDevicePath (
  IN OUT EFI_DEVICE_PATH_PROTOCOL  **DevicePath,
  OUT UINTN                        *FileSize
  )
{
  UINTN                             ParentDevicePathSize;
  EFI_DEVICE_PATH_PROTOCOL          *ShortformNode;
  EFI_DEVICE_PATH_PROTOCOL          *RemainingDevicePath;
  EFI_DEVICE_PATH_PROTOCOL          *FullDevicePath;
  EFI_HANDLE                        *UsbIoHandles;
  UINTN                             UsbIoHandleCount;
  UINTN                             Index;
  VOID                              *FileBuffer;
  
  //
  // Search for USB Class or USB WWID device path node.
  //
  for ( ShortformNode = *DevicePath
      ; !IsDevicePathEnd (ShortformNode)
      ; ShortformNode = NextDevicePathNode (ShortformNode)
      ) {
    if ((DevicePathType (ShortformNode) == MESSAGING_DEVICE_PATH) &&
        ((DevicePathSubType (ShortformNode) == MSG_USB_CLASS_DP) ||
         (DevicePathSubType (ShortformNode) == MSG_USB_WWID_DP))) {
      break;
    }
  }
  ASSERT (!IsDevicePathEnd (ShortformNode));

  FullDevicePath       = NULL;
  ParentDevicePathSize = (UINTN) ShortformNode - (UINTN) *DevicePath;
  RemainingDevicePath  = NextDevicePathNode (ShortformNode);
  FileBuffer           = NULL;
  UsbIoHandles         = FindUsbDevice (*DevicePath, ParentDevicePathSize, &UsbIoHandleCount);

  for (Index = 0; Index < UsbIoHandleCount; Index++) {
    FullDevicePath = AppendDevicePath (DevicePathFromHandle (UsbIoHandles[Index]), RemainingDevicePath);
    DEBUG ((EFI_D_INFO, "[Bds] FullDp1[%d]:", Index)); DEBUG_CODE (PrintDp (FullDevicePath); ); DEBUG ((EFI_D_INFO, "\n"));
    FileBuffer = LoadEfiBootOption (&FullDevicePath, FileSize);
    if (FileBuffer != NULL) {
      DEBUG ((EFI_D_INFO, "-->")); DEBUG_CODE (PrintDp (FullDevicePath); ); DEBUG ((EFI_D_INFO, FileBuffer != NULL ? " - Found\n" : "\n"));
      break;
    }
  }

  if (UsbIoHandles != NULL) {
    FreePool (UsbIoHandles);
  }

  if (FileBuffer == NULL) {
    //
    // Boot Option device path starts with USB Class or USB WWID device path.
    // For Boot Option device path which doesn't begin with the USB Class or
    // USB WWID device path, it's not needed to connect again here.
    //
    if ((DevicePathType (*DevicePath) == MESSAGING_DEVICE_PATH) &&
        ((DevicePathSubType (*DevicePath) == MSG_USB_CLASS_DP) ||
         (DevicePathSubType (*DevicePath) == MSG_USB_WWID_DP))) {
      EfiBootManagerConnectUsbShortFormDevicePath (*DevicePath);

      UsbIoHandles = FindUsbDevice (*DevicePath, ParentDevicePathSize, &UsbIoHandleCount);
      for (Index = 0; Index < UsbIoHandleCount; Index++) {
        FullDevicePath = AppendDevicePath (DevicePathFromHandle (UsbIoHandles[Index]), RemainingDevicePath);
        DEBUG ((EFI_D_INFO, "[Bds] FullDp2[%d]:", Index)); DEBUG_CODE (PrintDp (FullDevicePath); ); DEBUG ((EFI_D_INFO, "\n"));
        FileBuffer = LoadEfiBootOption (&FullDevicePath, FileSize);
        if (FileBuffer != NULL) {
          DEBUG ((EFI_D_INFO, "-->")); DEBUG_CODE (PrintDp (FullDevicePath); ); DEBUG ((EFI_D_INFO, FileBuffer != NULL ? " - Found\n" : "\n"));
          break;
        }
      }

      if (UsbIoHandles != NULL) {
        FreePool (UsbIoHandles);
      }
    }
  }

  FreeAndSet ((VOID **) DevicePath, FullDevicePath);
  return FileBuffer;
}

/**
  Expand a device path that starts with a hard drive media device path node to be a
  full device path that includes the full hardware path to the device. We need
  to do this so it can be booted. As an optimization the front match (the part point
  to the partition node. E.g. ACPI() /PCI()/ATA()/Partition() ) is saved in a variable
  so a connect all is not required on every boot. All successful history device path
  which point to partition node (the front part) will be saved.

  @param  DevicePath    On input, a pointer to an allocated buffer that contains the 
                        file device path.
                        On output, a pointer to an reallocated buffer that contains 
                        the expanded device path. It would point to NULL if the file
                        cannot be read.

**/
VOID
ExpandPartitionShortFormDevicePath (
  IN OUT EFI_DEVICE_PATH_PROTOCOL      **DevicePath
  )
{
  EFI_STATUS                Status;
  UINTN                     BlockIoHandleCount;
  EFI_HANDLE                *BlockIoBuffer;
  EFI_DEVICE_PATH_PROTOCOL  *FullDevicePath;
  EFI_DEVICE_PATH_PROTOCOL  *BlockIoDevicePath;
  UINTN                     Index;
  UINTN                     InstanceNum;
  EFI_DEVICE_PATH_PROTOCOL  *CachedDevicePath;
  EFI_DEVICE_PATH_PROTOCOL  *TempNewDevicePath;
  UINTN                     CachedDevicePathSize;
  BOOLEAN                   DeviceExist;
  BOOLEAN                   NeedAdjust;
  EFI_DEVICE_PATH_PROTOCOL  *Instance;
  UINTN                     Size;

  FullDevicePath      = NULL;
  //
  // Check if there is prestore 'HDDP' variable.
  // If exist, search the front path which point to partition node in the variable instants.
  // If fail to find or 'HDDP' not exist, reconnect all and search in all system
  //
  CachedDevicePath = EfiBootManagerGetVariableAndSize (
                      L"HDDP",
                      &mHdBootVariablePrivateGuid,
                      &CachedDevicePathSize
                      );

  if (CachedDevicePath != NULL) {
    TempNewDevicePath = CachedDevicePath;
    DeviceExist = FALSE;
    NeedAdjust = FALSE;
    do {
      //
      // Check every instance of the variable
      // First, check whether the instance contain the partition node, which is needed for distinguishing  multi
      // partial partition boot option. Second, check whether the instance could be connected.
      //
      Instance  = GetNextDevicePathInstance (&TempNewDevicePath, &Size);
      if (MatchPartitionDevicePathNode (Instance, (HARDDRIVE_DEVICE_PATH *) *DevicePath)) {
        //
        // Connect the device path instance, the device path point to hard drive media device path node
        // e.g. ACPI() /PCI()/ATA()/Partition()
        //
        Status = EfiBootManagerConnectDevicePath (Instance, NULL);
        if (!EFI_ERROR (Status)) {
          DeviceExist = TRUE;
          break;
        }
      }
      //
      // Come here means the first instance is not matched
      //
      NeedAdjust = TRUE;
      FreePool(Instance);
    } while (TempNewDevicePath != NULL);

    if (DeviceExist) {
      //
      // Find the matched device path.
      // Append the file path information from the boot option and return the fully expanded device path.
      //
      FullDevicePath = AppendDevicePath (Instance, NextDevicePathNode (*DevicePath));

      //
      // Adjust the 'HDDP' instances sequence if the matched one is not first one.
      //
      if (NeedAdjust) {
        //
        // First delete the matched instance.
        //
        TempNewDevicePath = CachedDevicePath;
        CachedDevicePath  = EfiBootManagerDelPartMatchInstance (CachedDevicePath, Instance);
        FreePool (TempNewDevicePath);

        //
        // Second, append the remaining path after the matched instance
        //
        TempNewDevicePath = CachedDevicePath;
        CachedDevicePath = AppendDevicePathInstance (Instance, CachedDevicePath );
        FreePool (TempNewDevicePath);
        //
        // Save the matching Device Path so we don't need to do a connect all next time
        //
        Status = gRT->SetVariable (
                        L"HDDP",
                        &mHdBootVariablePrivateGuid,
                        EFI_VARIABLE_BOOTSERVICE_ACCESS | EFI_VARIABLE_RUNTIME_ACCESS | EFI_VARIABLE_NON_VOLATILE,
                        GetDevicePathSize (CachedDevicePath),
                        CachedDevicePath
                        );
      }

      FreePool (Instance);
      FreePool (CachedDevicePath);
      FreePool (*DevicePath);
      *DevicePath = FullDevicePath;
      return;
    }
  }

  //
  // If we get here we fail to find or 'HDDP' not exist, and now we need
  // to search all devices in the system for a matched partition
  //
  EfiBootManagerConnectAll ();
  Status = gBS->LocateHandleBuffer (ByProtocol, &gEfiBlockIoProtocolGuid, NULL, &BlockIoHandleCount, &BlockIoBuffer);
  if (EFI_ERROR (Status)) {
    BlockIoHandleCount = 0;
    BlockIoBuffer      = NULL;
  }
  //
  // Loop through all the device handles that support the BLOCK_IO Protocol
  //
  for (Index = 0; Index < BlockIoHandleCount; Index++) {

    Status = gBS->HandleProtocol (BlockIoBuffer[Index], &gEfiDevicePathProtocolGuid, (VOID *) &BlockIoDevicePath);
    if (EFI_ERROR (Status) || BlockIoDevicePath == NULL) {
      continue;
    }

    if (MatchPartitionDevicePathNode (BlockIoDevicePath, (HARDDRIVE_DEVICE_PATH *) *DevicePath)) {
      //
      // Find the matched partition device path
      //
      FullDevicePath = AppendDevicePath (BlockIoDevicePath, NextDevicePathNode (*DevicePath));

      //
      // Save the matched partition device path in 'HDDP' variable
      //
      if (CachedDevicePath != NULL) {
        //
        // Save the matched partition device path as first instance of 'HDDP' variable
        //
        if (EfiBootManagerMatchDevicePaths (CachedDevicePath, BlockIoDevicePath)) {
          TempNewDevicePath = CachedDevicePath;
          CachedDevicePath = EfiBootManagerDelPartMatchInstance (CachedDevicePath, BlockIoDevicePath);
          FreePool(TempNewDevicePath);

          TempNewDevicePath = CachedDevicePath;
          CachedDevicePath = AppendDevicePathInstance (BlockIoDevicePath, CachedDevicePath);
          if (TempNewDevicePath != NULL) {
            FreePool(TempNewDevicePath);
          }
        } else {
          TempNewDevicePath = CachedDevicePath;
          CachedDevicePath = AppendDevicePathInstance (BlockIoDevicePath, CachedDevicePath);
          FreePool(TempNewDevicePath);
        }
        //
        // Here limit the device path instance number to 12, which is max number for a system support 3 IDE controller
        // If the user try to boot many OS in different HDs or partitions, in theory, the 'HDDP' variable maybe become larger and larger.
        //
        InstanceNum = 0;
        ASSERT (CachedDevicePath != NULL);
        TempNewDevicePath = CachedDevicePath;
        while (!IsDevicePathEnd (TempNewDevicePath)) {
          TempNewDevicePath = NextDevicePathNode (TempNewDevicePath);
          //
          // Parse one instance
          //
          while (!IsDevicePathEndType (TempNewDevicePath)) {
            TempNewDevicePath = NextDevicePathNode (TempNewDevicePath);
          }
          InstanceNum++;
          //
          // If the CachedDevicePath variable contain too much instance, only remain 12 instances.
          //
          if (InstanceNum >= 12) {
            SetDevicePathEndNode (TempNewDevicePath);
            break;
          }
        }
      } else {
        CachedDevicePath = DuplicateDevicePath (BlockIoDevicePath);
      }

      //
      // Save the matching Device Path so we don't need to do a connect all next time
      //
      Status = gRT->SetVariable (
                      L"HDDP",
                      &mHdBootVariablePrivateGuid,
                      EFI_VARIABLE_BOOTSERVICE_ACCESS | EFI_VARIABLE_RUNTIME_ACCESS | EFI_VARIABLE_NON_VOLATILE,
                      GetDevicePathSize (CachedDevicePath),
                      CachedDevicePath
                      );

      break;
    }
  }

  if (CachedDevicePath != NULL) {
    FreePool (CachedDevicePath);
  }
  if (BlockIoBuffer != NULL) {
    FreePool (BlockIoBuffer);
  }
  FreeAndSet ((VOID **) DevicePath, FullDevicePath);
}

/**
  Algorithm follows the UEFI Spec chapter 3.4 Boot Mechanisms.

  @param  DevicePath  Device Path to a  bootable device

  @return  The bootable media handle. If the media on the DevicePath is not bootable, NULL will return.

**/
EFI_HANDLE
GetBootableDeviceHandle (
  IN  EFI_DEVICE_PATH_PROTOCOL        *DevicePath
  )
{
  EFI_STATUS                          Status;
  EFI_DEVICE_PATH_PROTOCOL            *UpdatedDevicePath;
  EFI_HANDLE                          Handle;
  EFI_BLOCK_IO_PROTOCOL               *BlockIo;
  VOID                                *Buffer;
  EFI_DEVICE_PATH_PROTOCOL            *TempDevicePath;
  UINTN                               Size;
  UINTN                               TempSize;
  EFI_HANDLE                          ReturnHandle;
  EFI_HANDLE                          *SimpleFileSystemHandles;
  UINTN                               NumberSimpleFileSystemHandles;
  UINTN                               Index;
  EFI_IMAGE_DOS_HEADER                DosHeader;
  EFI_IMAGE_OPTIONAL_HEADER_UNION     HdrData;
  EFI_IMAGE_OPTIONAL_HEADER_PTR_UNION Hdr;

  ReturnHandle      = NULL;
  UpdatedDevicePath = DevicePath;

  //
  // Check whether the device is connected
  //
  Status = gBS->LocateDevicePath (&gEfiBlockIoProtocolGuid, &UpdatedDevicePath, &Handle);
  if (EFI_ERROR (Status)) {
    //
    // Skip the case that the boot option point to a simple file protocol which does not consume block Io protocol,
    //
    Status = gBS->LocateDevicePath (&gEfiSimpleFileSystemProtocolGuid, &UpdatedDevicePath, &Handle);
    if (EFI_ERROR (Status)) {
      //
      // Fail to find the proper BlockIo and simple file protocol, maybe because device not present,  we need to connect it firstly
      //
      UpdatedDevicePath = DevicePath;
      Status            = gBS->LocateDevicePath (&gEfiDevicePathProtocolGuid, &UpdatedDevicePath, &Handle);
      gBS->ConnectController (Handle, NULL, NULL, TRUE);
    }
  } else {
    //
    // For removable device boot option, its contained device path only point to the removable device handle, 
    // should make sure all its children handles (its child partion or media handles) are created and connected. 
    //
    gBS->ConnectController (Handle, NULL, NULL, TRUE); 
    //
    // Get BlockIo protocol and check removable attribute
    //
    Status = gBS->HandleProtocol (Handle, &gEfiBlockIoProtocolGuid, (VOID **)&BlockIo);
    //
    // Issue a dummy read to the device to check for media change.
    // When the removable media is changed, any Block IO read/write will
    // cause the BlockIo protocol be reinstalled and EFI_MEDIA_CHANGED is
    // returned. After the Block IO protocol is reinstalled, subsequent
    // Block IO read/write will success.
    //
    Buffer = AllocatePool (BlockIo->Media->BlockSize);
    if (Buffer != NULL) {
      BlockIo->ReadBlocks (
               BlockIo,
               BlockIo->Media->MediaId,
               0,
               BlockIo->Media->BlockSize,
               Buffer
               );
      FreePool(Buffer);
    }
  }

  //
  // Detect the the default boot file from removable Media
  //
  Size = GetDevicePathSize(DevicePath) - END_DEVICE_PATH_LENGTH;
  gBS->LocateHandleBuffer (
         ByProtocol,
         &gEfiSimpleFileSystemProtocolGuid,
         NULL,
         &NumberSimpleFileSystemHandles,
         &SimpleFileSystemHandles
         );
  for (Index = 0; Index < NumberSimpleFileSystemHandles; Index++) {
    //
    // Get the device path size of SimpleFileSystem handle
    //
    TempDevicePath = DevicePathFromHandle (SimpleFileSystemHandles[Index]);
    TempSize       = GetDevicePathSize (TempDevicePath)- END_DEVICE_PATH_LENGTH;
    //
    // Check whether the device path of boot option is part of the SimpleFileSystem handle's device path
    //
    if ((Size <= TempSize) && (CompareMem (TempDevicePath, DevicePath, Size) == 0)) {
      //
      // Load the default boot file \EFI\BOOT\boot{machinename}.EFI from removable Media
      //  machinename is ia32, ia64, x64, ...
      //
      Hdr.Union = &HdrData;
      Status = GetImageHeader (
                 SimpleFileSystemHandles[Index],
                 EFI_REMOVABLE_MEDIA_FILE_NAME,
                 &DosHeader,
                 Hdr
                 );
      if (!EFI_ERROR (Status) && EFI_IMAGE_MACHINE_TYPE_SUPPORTED (Hdr.Pe32->FileHeader.Machine) &&
          (Hdr.Pe32->OptionalHeader.Subsystem == EFI_IMAGE_SUBSYSTEM_EFI_APPLICATION)
         ) {
        ReturnHandle = SimpleFileSystemHandles[Index];
        break;
      }
    }
  }

  if (SimpleFileSystemHandles != NULL) {
    FreePool(SimpleFileSystemHandles);
  }

  return ReturnHandle;
}

/**
  Get the image file buffer data and buffer size by its device path. 

  @param FilePath  On input, a pointer to an allocated buffer that contains the 
                   file device path.
                   On output the device path pointer could be modified to point to
                   a new allocated buffer that contains the full device path.
                   It could be caused by either short-form device path expanding,
                   or default boot file path appending.
  @param FileSize  A pointer to the size of the file buffer.

  @retval NULL   The file can't be found.
  @retval other  The file buffer. The caller is responsible to free memory.
**/
VOID *
LoadEfiBootOption (
  IN OUT EFI_DEVICE_PATH_PROTOCOL **FilePath,
  OUT    UINTN                    *FileSize
  )
{
  EFI_HANDLE                      Handle;
  VOID                            *FileBuffer;
  UINT32                          AuthenticationStatus;
  EFI_DEVICE_PATH_PROTOCOL        *Node;

  ASSERT ((FilePath != NULL) && (*FilePath != NULL) && (FileSize != NULL));

  EfiBootManagerConnectDevicePath (*FilePath, NULL);

  *FileSize  = 0;
  FileBuffer = NULL;
  //
  // Expand the short-form device path to full device path
  //
  if (FeaturePcdGet (PcdShortformBootSupport)) {

    if ((DevicePathType (*FilePath) == MEDIA_DEVICE_PATH) &&
        (DevicePathSubType (*FilePath) == MEDIA_HARDDRIVE_DP)) {
      //
      // Expand the Harddrive device path
      //
      ExpandPartitionShortFormDevicePath (FilePath);
      if (*FilePath == NULL) {
        return NULL;
      }

    } else {
      for (Node = *FilePath; !IsDevicePathEnd (Node); Node = NextDevicePathNode (Node)) {
        if ((DevicePathType (Node) == MESSAGING_DEVICE_PATH) &&
            ((DevicePathSubType (Node) == MSG_USB_CLASS_DP) ||
             (DevicePathSubType (Node) == MSG_USB_WWID_DP))) {
          break;
        }
      }

      if (!IsDevicePathEnd (Node)) {
        //
        // Expand the USB WWID/Class device path
        //
        FileBuffer = ExpandUsbShortFormDevicePath (FilePath, FileSize);
        if (FileBuffer == NULL) {
          return NULL;
        }
      }
    }
  }

  //
  // Fix up the boot option path if it points to a FV in memory map style of device path
  //
  if (IsMemmapFvFilePath (*FilePath)) {
    FixupMemmapFvFilePath (FilePath);
    if (*FilePath == NULL) {
      return NULL;
    }
  }

  if (FileBuffer == NULL) {
    FileBuffer = GetFileBufferByFilePath (TRUE, *FilePath, FileSize, &AuthenticationStatus);
  }

  //
  // If we didn't find an image directly, we need to try as if it is a removable device boot option
  // and load the image according to the default boot behavior.
  //
  if (FileBuffer == NULL) {
    //
    // check if there is a bootable media could be found in this device path,
    // and get the bootable media handle
    //
    Handle = GetBootableDeviceHandle (*FilePath);
    if (Handle != NULL) {
      //
      // Load the default boot file \EFI\BOOT\boot{machinename}.EFI from the media
      //  machinename is ia32, ia64, x64, ...
      //
      FreeAndSet ((VOID **) FilePath, FileDevicePath (Handle, EFI_REMOVABLE_MEDIA_FILE_NAME));
      ASSERT (*FilePath != NULL);
      FileBuffer = GetFileBufferByFilePath (TRUE, *FilePath, FileSize, &AuthenticationStatus);
    }
  }

  if (FileBuffer == NULL) {
    FreeAndSet ((VOID **) FilePath, NULL);
  }

  return FileBuffer;
}

/**
  Attempt to boot the EFI boot option. This routine sets L"BootCurent" and
  also signals the EFI ready to boot event. If the device path for the option
  starts with a BBS device path a legacy boot is attempted via the registered 
  gLegacyBoot function. Short form device paths are also supported via this 
  rountine. A device path starting with MEDIA_HARDDRIVE_DP, MSG_USB_WWID_DP,
  MSG_USB_CLASS_DP gets expaned out to find the first device that matches.
  If the BootOption Device Path fails the removable media boot algorithm 
  is attempted (\EFI\BOOTIA32.EFI, \EFI\BOOTX64.EFI,... only one file type 
  is tried per processor type)

  @param  BootOption    Boot Option to try and boot.
                        On return, BootOption->Status contains the boot status.
                        EFI_SUCCESS     BootOption was booted
                        EFI_UNSUPPORTED A BBS device path was found with no valid callback
                                        registered via EfiBootManagerInitialize().
                        EFI_NOT_FOUND   The BootOption was not found on the system
                        !EFI_SUCCESS    BootOption failed with this error status

**/
VOID
EFIAPI
EfiBootManagerBoot (
  IN  EFI_BOOT_MANAGER_LOAD_OPTION             *BootOption
  )
{
  EFI_STATUS                Status;
  EFI_HANDLE                ImageHandle;
  EFI_LOADED_IMAGE_PROTOCOL *ImageInfo;
  UINT16                    OptionNumber;
  EFI_DEVICE_PATH_PROTOCOL  *FilePath;
  VOID                      *FileBuffer;
  UINTN                     FileSize;
  EFI_BOOT_LOGO_PROTOCOL    *BootLogo;
  EFI_EVENT                 LegacyBootEvent;

  if (BootOption == NULL) {
    return;
  }

  if (BootOption->FilePath == NULL) {
    BootOption->Status = EFI_INVALID_PARAMETER;
    return;
  }

  //
  // 1. Create Boot#### for a temporary boot (i.e. a boot by selected a EFI Shell using "Boot From File")
  //
  if (!BootOptionInVariable (BootOption) && (BootOption->OptionNumber == LoadOptionNumberUnassigned)) {
    Status = GetFreeOptionNumber (L"BootOrder", &OptionNumber);
    if (!EFI_ERROR (Status)) {
      BootOption->OptionNumber = OptionNumber;
      Status = EfiBootManagerLoadOptionToVariable (BootOption);
    }
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "[Bds] Failed to create Boot#### for a temporary boot - %r!\n", Status));
      BootOption->Status = Status;
      return ;
    }
  }

  //
  // 2. Update system variables: remove BootNext, set BootCurrent
  //

  //
  // 2.1. Remove BootNext
  // To prevent loops, the boot manager deletes BootNext before transferring control to the 
  // preselected boot option.
  //
  if (BootOption->BootNext) {
    Status = gRT->SetVariable (
                    L"BootNext",
                    &gEfiGlobalVariableGuid,
                    0,
                    0,
                    NULL
                    );
    ASSERT_EFI_ERROR (Status);
  }
  //
  // 2.2. Set BootCurrent
  //
  Status = gRT->SetVariable (
                  L"BootCurrent",
                  &gEfiGlobalVariableGuid,
                  EFI_VARIABLE_BOOTSERVICE_ACCESS | EFI_VARIABLE_RUNTIME_ACCESS,
                  sizeof (UINT16),
                  &BootOption->OptionNumber
                  );
  ASSERT_EFI_ERROR (Status);

  //
  // Report Status Code to indicate ReadyToBoot event will be signalled
  //
  REPORT_STATUS_CODE (EFI_PROGRESS_CODE, (EFI_SOFTWARE_DXE_BS_DRIVER | EFI_SW_DXE_BS_PC_READY_TO_BOOT_EVENT));

  //
  // 3. Signal the EVT_SIGNAL_READY_TO_BOOT event when we are about to load and execute
  //    the boot option.
  //
  EfiSignalEventReadyToBoot();

  PERF_START_EX (gImageHandle, "BdsAttempt", NULL, 0, (UINT32) BootOption->OptionNumber);

  //
  // 4. Load EFI boot option to ImageHandle
  //
  ImageHandle = NULL;
  if (DevicePathType (BootOption->FilePath) != BBS_DEVICE_PATH) {
    Status     = EFI_NOT_FOUND;
    FilePath   = DuplicateDevicePath (BootOption->FilePath);
    FileBuffer = LoadEfiBootOption (&FilePath, &FileSize);
    if (FileBuffer != NULL) {

      REPORT_STATUS_CODE (EFI_PROGRESS_CODE, PcdGet32 (PcdProgressCodeOsLoaderLoad));

      Status = gBS->LoadImage (
                      TRUE,
                      gImageHandle,
                      FilePath,
                      FileBuffer,
                      FileSize,
                      &ImageHandle
                      );
      FreePool (FileBuffer);
      FreePool (FilePath);
    }

    if (EFI_ERROR (Status)) {
      //
      // Report Status Code to indicate that the failure to load boot option
      //
      REPORT_STATUS_CODE (
        EFI_ERROR_CODE | EFI_ERROR_MINOR,
        (EFI_SOFTWARE_DXE_BS_DRIVER | EFI_SW_DXE_BS_EC_BOOT_OPTION_LOAD_ERROR)
        );
      BootOption->Status = Status;
      return;
    }
  }

  PERF_CODE (
    AllocateMemoryForPerformanceData ();
  );

  //
  // 5. Adjust the different type memory page number just before booting
  //    and save the updated info into the variable for next boot to use
  //
  if ((BootOption->Attributes & LOAD_OPTION_CATEGORY) == LOAD_OPTION_CATEGORY_BOOT) {
    if (PcdGetBool (PcdResetOnMemoryTypeInformationChange)) {
      SetMemoryTypeInformationVariable ();
    }
  }

  DEBUG_CODE_BEGIN();
    if (BootOption->Description == NULL) {
      DEBUG ((DEBUG_INFO | DEBUG_LOAD, "[Bds]Booting from unknown device path\n"));
    } else {
      DEBUG ((DEBUG_INFO | DEBUG_LOAD, "[Bds]Booting %s\n", BootOption->Description));
    }
  DEBUG_CODE_END();

  //
  // Check to see if we should legacy BOOT. If yes then do the legacy boot
  // Write boot to OS performance data for Legacy boot
  //
  if ((DevicePathType (BootOption->FilePath) == BBS_DEVICE_PATH) && (DevicePathSubType (BootOption->FilePath) == BBS_BBS_DP)) {
    if (mEfiBootManagerLegacyBoot != NULL) {
      //
      // Write boot to OS performance data for legacy boot.
      //
      PERF_CODE (
        //
        // Create an event to be signalled when Legacy Boot occurs to write performance data.
        //
        Status = EfiCreateEventLegacyBootEx(
                   TPL_NOTIFY,
                   WriteBootToOsPerformanceData,
                   NULL, 
                   &LegacyBootEvent
                   );
        ASSERT_EFI_ERROR (Status);
      );

      mEfiBootManagerLegacyBoot (BootOption);
    } else {
      BootOption->Status = EFI_UNSUPPORTED;
    }

    PERF_CODE (
      FreeMemoryForPerformanceData ();
    );
    PERF_END_EX (gImageHandle, "BdsAttempt", NULL, 0, (UINT32) BootOption->OptionNumber);
    return;
  }
 
  //
  // Provide the image with its load options
  //
  Status = gBS->HandleProtocol (ImageHandle, &gEfiLoadedImageProtocolGuid, (VOID **) &ImageInfo);
  ASSERT_EFI_ERROR (Status);

  ImageInfo->LoadOptionsSize  = BootOption->OptionalDataSize;
  ImageInfo->LoadOptions      = BootOption->OptionalData;

  //
  // Clean to NULL because the image is loaded directly from the firmwares boot manager.
  //
  ImageInfo->ParentHandle = NULL;

  //
  // Before calling the image, enable the Watchdog Timer for 5 minutes period
  //
  gBS->SetWatchdogTimer (5 * 60, 0x0000, 0x00, NULL);

  //
  // Write boot to OS performance data for UEFI boot
  //
  PERF_CODE (
    WriteBootToOsPerformanceData (NULL, NULL);
  );

  REPORT_STATUS_CODE (EFI_PROGRESS_CODE, PcdGet32 (PcdProgressCodeOsLoaderStart));

  Status = gBS->StartImage (ImageHandle, &BootOption->ExitDataSize, &BootOption->ExitData);
  DEBUG ((DEBUG_INFO | DEBUG_LOAD, "Image Return Status = %r\n", Status));
  BootOption->Status = Status;
  if (EFI_ERROR (Status)) {
    //
    // Report Status Code to indicate that boot failure
    //
    REPORT_STATUS_CODE (
      EFI_ERROR_CODE | EFI_ERROR_MINOR,
      (EFI_SOFTWARE_DXE_BS_DRIVER | EFI_SW_DXE_BS_EC_BOOT_OPTION_FAILED)
      );
  }
  PERF_END_EX (gImageHandle, "BdsAttempt", NULL, 0, (UINT32) BootOption->OptionNumber);

  //
  // Clear the Watchdog Timer after the image returns
  //
  gBS->SetWatchdogTimer (0x0000, 0x0000, 0x0000, NULL);

  //
  // Set Logo status invalid after trying one boot option
  //
  BootLogo = NULL;
  Status = gBS->LocateProtocol (&gEfiBootLogoProtocolGuid, NULL, (VOID **) &BootLogo);
  if (!EFI_ERROR (Status) && (BootLogo != NULL)) {
    Status = BootLogo->SetBootLogo (BootLogo, NULL, 0, 0, 0, 0);
    ASSERT_EFI_ERROR (Status);
  }

  //
  // Clear Boot Current
  //
  gRT->SetVariable (
        L"BootCurrent",
        &gEfiGlobalVariableGuid,
        0,
        0,
        NULL
        );
  PERF_CODE (
    FreeMemoryForPerformanceData ();
  );
}


/**
  Check whether there is a instance in BlockIoDevicePath, which contain multi device path
  instances, has the same partition node with HardDriveDevicePath device path

  @param  BlockIoDevicePath      Multi device path instances which need to check
  @param  HardDriveDevicePath    A device path which starts with a hard drive media
                                 device path.

  @retval TRUE                   There is a matched device path instance.
  @retval FALSE                  There is no matched device path instance.

**/
BOOLEAN
MatchPartitionDevicePathNode (
  IN  EFI_DEVICE_PATH_PROTOCOL   *BlockIoDevicePath,
  IN  HARDDRIVE_DEVICE_PATH      *HardDriveDevicePath
  )
{
  HARDDRIVE_DEVICE_PATH     *TmpHdPath;
  EFI_DEVICE_PATH_PROTOCOL  *DevicePath;
  BOOLEAN                   Match;
  EFI_DEVICE_PATH_PROTOCOL  *BlockIoHdDevicePathNode;

  if ((BlockIoDevicePath == NULL) || (HardDriveDevicePath == NULL)) {
    return FALSE;
  }

  //
  // Make PreviousDevicePath == the device path node before the end node
  //
  DevicePath              = BlockIoDevicePath;
  BlockIoHdDevicePathNode = NULL;

  //
  // find the partition device path node
  //
  while (!IsDevicePathEnd (DevicePath)) {
    if ((DevicePathType (DevicePath) == MEDIA_DEVICE_PATH) &&
        (DevicePathSubType (DevicePath) == MEDIA_HARDDRIVE_DP)
        ) {
      BlockIoHdDevicePathNode = DevicePath;
      break;
    }

    DevicePath = NextDevicePathNode (DevicePath);
  }

  if (BlockIoHdDevicePathNode == NULL) {
    return FALSE;
  }
  //
  // See if the harddrive device path in blockio matches the orig Hard Drive Node
  //
  TmpHdPath = (HARDDRIVE_DEVICE_PATH *) BlockIoHdDevicePathNode;
  Match = FALSE;

  //
  // Check for the match
  //
  if ((TmpHdPath->MBRType == HardDriveDevicePath->MBRType) &&
      (TmpHdPath->SignatureType == HardDriveDevicePath->SignatureType)) {
    switch (TmpHdPath->SignatureType) {
    case SIGNATURE_TYPE_GUID:
      Match = CompareGuid ((EFI_GUID *)TmpHdPath->Signature, (EFI_GUID *)HardDriveDevicePath->Signature);
      break;
    case SIGNATURE_TYPE_MBR:
      Match = (BOOLEAN) (*((UINT32 *) (&(TmpHdPath->Signature[0]))) == ReadUnaligned32((UINT32 *)(&(HardDriveDevicePath->Signature[0]))));
      break;
    default:
      Match = FALSE;
      break;
    }
  }

  return Match;
}

/**
  Emuerate all possible bootable medias in the following order:
  1. Removable BlockIo            - The boot option only points to the removable media
                                    device, like USB key, DVD, Floppy etc.
  2. Fixed BlockIo                - The boot option only points to a Fixed blockIo device,
                                    like HardDisk.
  3. Non-BlockIo SimpleFileSystem - The boot option points to a device supporting
                                    SimpleFileSystem Protocol, but not supporting BlockIo
                                    protocol.
  4. LoadFile                     - The boot option points to the media supporting 
                                    LoadFile protocol.
  Reference: UEFI Spec chapter 3.3 Boot Option Variables Default Boot Behavior

  TODO: Do we need to add EfiBootManagerConnectAll() in this function? Impact to SLE platform is big.
**/
EFI_BOOT_MANAGER_LOAD_OPTION *
EFIAPI
EfiBootManagerEnumerateBootOptions (
  UINTN                                 *BootOptionCount
  )
{
  EFI_STATUS                            Status;
  EFI_BOOT_MANAGER_LOAD_OPTION          *BootOptions;
  UINT16                                NonBlockNumber;
  UINTN                                 HandleCount;
  EFI_HANDLE                            *Handles;
  EFI_BLOCK_IO_PROTOCOL                 *BlkIo;
  UINTN                                 Removable;
  UINTN                                 Index;
  UINTN                                 FunctionIndex;
  CHAR16                                *Temp;
  CHAR16                                *DescriptionPtr;
  CHAR16                                Description[30];

  ASSERT (BootOptionCount != NULL);

  *BootOptionCount = 0;
  BootOptions      = NULL;

  //
  // Parse removable block io followed by fixed block io
  //
  gBS->LocateHandleBuffer (
         ByProtocol,
         &gEfiBlockIoProtocolGuid,
         NULL,
         &HandleCount,
         &Handles
         );

  for (Removable = 0; Removable < 2; Removable++) {
    for (Index = 0; Index < HandleCount; Index++) {
      Status = gBS->HandleProtocol (
                      Handles[Index],
                      &gEfiBlockIoProtocolGuid,
                      (VOID **) &BlkIo
                      );
      if (EFI_ERROR (Status)) {
        continue;
      }

      //
      // Skip the logical partitions
      //
      if (BlkIo->Media->LogicalPartition) {
        continue;
      }

      //
      // Skip the fixed block io then the removable block io
      //
      if (BlkIo->Media->RemovableMedia == ((Removable == 0) ? FALSE : TRUE)) {
        continue;
      }

      DescriptionPtr = NULL;
      for (FunctionIndex = 0; FunctionIndex < sizeof (mGetBootDescription) / sizeof (mGetBootDescription[0]); FunctionIndex++) {
        DescriptionPtr = mGetBootDescription[FunctionIndex] (Handles[Index]);
        if (DescriptionPtr != NULL) {
          break;
        }
      }

      if (DescriptionPtr == NULL) {
        continue;
      }

      //
      // Avoid description confusion between UEFI & Legacy boot option by adding "UEFI " prefix
      //
      Temp = AllocatePool (StrSize (DescriptionPtr) + sizeof (mUefiPrefix)); 
      ASSERT (Temp != NULL);
      StrCpy (Temp, mUefiPrefix);
      StrCat (Temp, DescriptionPtr);
      FreePool (DescriptionPtr);
      DescriptionPtr = Temp;

      BootOptions = ReallocatePool (
                      sizeof (EFI_BOOT_MANAGER_LOAD_OPTION) * (*BootOptionCount),
                      sizeof (EFI_BOOT_MANAGER_LOAD_OPTION) * (*BootOptionCount + 1),
                      BootOptions
                      );
      ASSERT (BootOptions != NULL);

      Status = EfiBootManagerInitializeLoadOption (
                 &BootOptions[(*BootOptionCount)++],
                 LoadOptionNumberUnassigned,
                 LoadOptionTypeBoot,
                 LOAD_OPTION_ACTIVE,
                 DescriptionPtr,
                 DevicePathFromHandle (Handles[Index]),
                 NULL,
                 0
                 );
      ASSERT_EFI_ERROR (Status);

      FreePool (DescriptionPtr);
    }
  }

  if (HandleCount != 0) {
    FreePool (Handles);
  }

  //
  // Parse simple file system not based on block io
  //
  NonBlockNumber = 0;
  gBS->LocateHandleBuffer (
         ByProtocol,
         &gEfiSimpleFileSystemProtocolGuid,
         NULL,
         &HandleCount,
         &Handles
         );
  for (Index = 0; Index < HandleCount; Index++) {
    Status = gBS->HandleProtocol (
                    Handles[Index],
                    &gEfiBlockIoProtocolGuid,
                    (VOID **) &BlkIo
                    );
     if (!EFI_ERROR (Status)) {
      //
      //  Skip if the file system handle supports a BlkIo protocol, which we've handled in above
      //
      continue;
    }
    UnicodeSPrint (Description, sizeof (Description), NonBlockNumber > 0 ? L"%s %d" : L"%s", L"UEFI Non-Block Boot Device", NonBlockNumber);
    
    BootOptions = ReallocatePool (
                    sizeof (EFI_BOOT_MANAGER_LOAD_OPTION) * (*BootOptionCount),
                    sizeof (EFI_BOOT_MANAGER_LOAD_OPTION) * (*BootOptionCount + 1),
                    BootOptions
                    );
    ASSERT (BootOptions != NULL);

    Status = EfiBootManagerInitializeLoadOption (
               &BootOptions[(*BootOptionCount)++],
               LoadOptionNumberUnassigned,
               LoadOptionTypeBoot,
               LOAD_OPTION_ACTIVE,
               Description,
               DevicePathFromHandle (Handles[Index]),
               NULL,
               0
               );
    ASSERT_EFI_ERROR (Status);
  }

  if (HandleCount != 0) {
    FreePool (Handles);
  }

  //
  // Parse load file, assuming UEFI Network boot option
  //
  gBS->LocateHandleBuffer (
         ByProtocol,
         &gEfiLoadFileProtocolGuid,
         NULL,
         &HandleCount,
         &Handles
         );
  for (Index = 0; Index < HandleCount; Index++) {

    UnicodeSPrint (Description, sizeof (Description), Index > 0 ? L"%s %d" : L"%s", L"UEFI Network", Index);

    BootOptions = ReallocatePool (
                    sizeof (EFI_BOOT_MANAGER_LOAD_OPTION) * (*BootOptionCount),
                    sizeof (EFI_BOOT_MANAGER_LOAD_OPTION) * (*BootOptionCount + 1),
                    BootOptions
                    );
    ASSERT (BootOptions != NULL);

    Status = EfiBootManagerInitializeLoadOption (
               &BootOptions[(*BootOptionCount)++],
               LoadOptionNumberUnassigned,
               LoadOptionTypeBoot,
               LOAD_OPTION_ACTIVE,
               Description,
               DevicePathFromHandle (Handles[Index]),
               NULL,
               0
               );
    ASSERT_EFI_ERROR (Status);
  }

  if (HandleCount != 0) {
    FreePool (Handles);
  }

  return BootOptions;
}

/**
  The function enumerates all boot options, creates them and registers them in the BootOrder variable.
**/
VOID
EFIAPI
EfiBootManagerRefreshAllBootOption (
  VOID
  )
{
  EFI_BOOT_MANAGER_LOAD_OPTION  *NvBootOptions;
  UINTN                         NvBootOptionCount;
  EFI_BOOT_MANAGER_LOAD_OPTION  *BootOptions;
  UINTN                         BootOptionCount;
  UINTN                         Index;

  //
  // Optionally refresh the legacy boot option
  //
  if (mEfiBootManagerRefreshLegacyBootOption != NULL) {
    mEfiBootManagerRefreshLegacyBootOption ();
  }

  BootOptions   = EfiBootManagerEnumerateBootOptions (&BootOptionCount);
  NvBootOptions = EfiBootManagerGetLoadOptions (&NvBootOptionCount, LoadOptionTypeBoot);

  //
  // Mark the boot option as added by bds by setting OptionalData to a special GUID
  //
  for (Index = 0; Index < BootOptionCount; Index++) {
    BootOptions[Index].OptionalData     = AllocateCopyPool (sizeof (EFI_GUID), &mAutoCreateBootOptionGuid);
    BootOptions[Index].OptionalDataSize = sizeof (EFI_GUID);
  }

  //
  // Remove invalid EFI boot options from NV
  //
  for (Index = 0; Index < NvBootOptionCount; Index++) {
    if (((DevicePathType (NvBootOptions[Index].FilePath) != BBS_DEVICE_PATH) || 
         (DevicePathSubType (NvBootOptions[Index].FilePath) != BBS_BBS_DP)
        ) &&
        (NvBootOptions[Index].OptionalDataSize == sizeof (EFI_GUID)) &&
        CompareGuid ((EFI_GUID *) NvBootOptions[Index].OptionalData, &mAutoCreateBootOptionGuid)
       ) {
      //
      // Only check those added by platform bds
      // so that the boot options added by end-user or OS installer won't be deleted
      //
      if (EfiBootManagerFindLoadOption (&NvBootOptions[Index], BootOptions, BootOptionCount) == (UINTN) -1) {
        EfiBootManagerDeleteLoadOptionVariable (NvBootOptions[Index].OptionNumber, LoadOptionTypeBoot);
      }
    }
  }

  //
  // Add new EFI boot options to NV
  //
  for (Index = 0; Index < BootOptionCount; Index++) {
    if (EfiBootManagerFindLoadOption (&BootOptions[Index], NvBootOptions, NvBootOptionCount) == (UINTN) -1) {
      EfiBootManagerAddLoadOptionVariable (&BootOptions[Index], (UINTN) -1);
    }
  }

  EfiBootManagerFreeLoadOptions (BootOptions,   BootOptionCount);
  EfiBootManagerFreeLoadOptions (NvBootOptions, NvBootOptionCount);
}


/**
  For a bootable Device path, return its boot type.

  @param  DevicePath                   The bootable device Path to check

  @retval AcpiFloppyBoot               If given device path contains ACPI_DEVICE_PATH type device path node
                                       which HID is floppy device.
  @retval MessageAtapiBoot             If given device path contains MESSAGING_DEVICE_PATH type device path node
                                       and its last device path node's subtype is MSG_ATAPI_DP.
  @retval MessageSataBoot              If given device path contains MESSAGING_DEVICE_PATH type device path node
                                       and its last device path node's subtype is MSG_SATA_DP.
  @retval MessageScsiBoot              If given device path contains MESSAGING_DEVICE_PATH type device path node
                                       and its last device path node's subtype is MSG_SCSI_DP.
  @retval MessageUsbBoot               If given device path contains MESSAGING_DEVICE_PATH type device path node
                                       and its last device path node's subtype is MSG_USB_DP.
  @retval MessageNetworkBoot           If given device path contains MESSAGING_DEVICE_PATH type device path node
                                       and its last device path node's subtype is MSG_MAC_ADDR_DP, MSG_VLAN_DP,
                                       MSG_IPv4_DP or MSG_IPv6_DP.
  @retval UnsupportedBoot              If tiven device path doesn't match the above condition, it's not supported.

**/
BOOT_TYPE
BootTypeFromDevicePath (
  IN  EFI_DEVICE_PATH_PROTOCOL     *DevicePath
  )
{
  EFI_DEVICE_PATH_PROTOCOL      *Node;
  EFI_DEVICE_PATH_PROTOCOL      *LastDeviceNode;

  ASSERT (DevicePath != NULL);

  for (Node = DevicePath; !IsDevicePathEndType (Node); Node = NextDevicePathNode (Node)) {
    switch (DevicePathType (Node)) {

      case ACPI_DEVICE_PATH:
        if (EISA_ID_TO_NUM (((ACPI_HID_DEVICE_PATH *) Node)->HID) == 0x0604) {
          return AcpiFloppyBoot;
        }
        break;

      case MESSAGING_DEVICE_PATH:
        //
        // Skip LUN device node
        //
        do {
          LastDeviceNode = NextDevicePathNode (Node);
        } while (DevicePathSubType(LastDeviceNode) == MSG_DEVICE_LOGICAL_UNIT_DP);

        //
        // if the device path not only point to driver device, it is not a messaging device path,
        //
        if (!IsDevicePathEndType (LastDeviceNode)) {
          break;
        }

        switch (DevicePathSubType (Node)) {
        case MSG_ATAPI_DP:
          return MessageAtapiBoot;
          break;

        case MSG_SATA_DP:
          return MessageSataBoot;
          break;

        case MSG_USB_DP:
          return MessageUsbBoot;
          break;

        case MSG_SCSI_DP:
          return MessageScsiBoot;
          break;

        case MSG_MAC_ADDR_DP:
        case MSG_VLAN_DP:
        case MSG_IPv4_DP:
        case MSG_IPv6_DP:
          return MessageNetworkBoot;
          break;
        }
    }
  }

  return UnsupportedBoot;
}

/**
  This function is called to create the boot option for the Boot Manager Menu.

  The Boot Manager Menu is shown after successfully booting a boot option.
  Assume the BootManagerMenuFile is in the same FV as the module links to this library.

  @param OptionNumber Return the boot option number of the Boot Manager Menu

  @retval EFI_SUCCESS   Successfully register the Boot Manager Menu.
  @retval Status        Return status of gRT->SetVariable ().
**/
EFI_STATUS
RegisterBootManagerMenu (
  OUT EFI_BOOT_MANAGER_LOAD_OPTION   *BootOption
  )
{
  EFI_STATUS                         Status;
  CHAR16                             *Description;
  UINTN                              DescriptionLength;
  EFI_DEVICE_PATH_PROTOCOL           *DevicePath;
  EFI_LOADED_IMAGE_PROTOCOL          *LoadedImage;
  MEDIA_FW_VOL_FILEPATH_DEVICE_PATH  FileNode;

  Status = GetSectionFromFv (
             PcdGetPtr (PcdBootManagerMenuFile),
             EFI_SECTION_USER_INTERFACE,
             0,
             (VOID **) &Description,
             &DescriptionLength
             );
  if (EFI_ERROR (Status)) {
    Description = NULL;
  }

  EfiInitializeFwVolDevicepathNode (&FileNode, PcdGetPtr (PcdBootManagerMenuFile));
  Status = gBS->HandleProtocol (
                  gImageHandle,
                  &gEfiLoadedImageProtocolGuid,
                  (VOID **) &LoadedImage
                  );
  ASSERT_EFI_ERROR (Status);
  DevicePath = AppendDevicePathNode (
                 DevicePathFromHandle (LoadedImage->DeviceHandle),
                 (EFI_DEVICE_PATH_PROTOCOL *) &FileNode
                 );
  ASSERT (DevicePath != NULL);

  Status = EfiBootManagerInitializeLoadOption (
             BootOption,
             LoadOptionNumberUnassigned,
             LoadOptionTypeBoot,
             LOAD_OPTION_CATEGORY_APP,
             (Description != NULL) ? Description : L"Boot Manager Menu",
             DevicePath,
             NULL,
             0
             );
  ASSERT_EFI_ERROR (Status);
  FreePool (DevicePath);
  if (Description != NULL) {
    FreePool (Description);
  }

  DEBUG_CODE (
    EFI_BOOT_MANAGER_LOAD_OPTION    *BootOptions;
    UINTN                           BootOptionCount;

    BootOptions = EfiBootManagerGetLoadOptions (&BootOptionCount, LoadOptionTypeBoot);
    ASSERT (EfiBootManagerFindLoadOption (BootOption, BootOptions, BootOptionCount) == -1);
    EfiBootManagerFreeLoadOptions (BootOptions, BootOptionCount);
    );

  return EfiBootManagerAddLoadOptionVariable (BootOption, 0);
}

/**
  Return the boot option corresponding to the Boot Manager Menu.
  It may automatically create one if the boot option hasn't been created yet.
  
  @param BootOption    Return the Boot Manager Menu.

  @retval EFI_SUCCESS   The Boot Manager Menu is successfully returned.
  @retval Status        Return status of gRT->SetVariable ().
**/
EFI_STATUS
EFIAPI
EfiBootManagerGetBootManagerMenu (
  EFI_BOOT_MANAGER_LOAD_OPTION *BootOption
  )
{
  EFI_STATUS                   Status;
  UINTN                        BootOptionCount;
  EFI_BOOT_MANAGER_LOAD_OPTION *BootOptions;
  UINTN                        Index;
  EFI_DEVICE_PATH_PROTOCOL     *Node;
  EFI_HANDLE                   FvHandle;
  
  BootOptions = EfiBootManagerGetLoadOptions (&BootOptionCount, LoadOptionTypeBoot);

  for (Index = 0; Index < BootOptionCount; Index++) {
    Node   = BootOptions[Index].FilePath;
    Status = gBS->LocateDevicePath (&gEfiFirmwareVolume2ProtocolGuid, &Node, &FvHandle);
    if (!EFI_ERROR (Status)) {
      if (CompareGuid (
            EfiGetNameGuidFromFwVolDevicePathNode ((CONST MEDIA_FW_VOL_FILEPATH_DEVICE_PATH *) Node),
            PcdGetPtr (PcdBootManagerMenuFile)
            )
          ) {        
        Status = EfiBootManagerInitializeLoadOption (
                   BootOption,
                   BootOptions[Index].OptionNumber,
                   BootOptions[Index].OptionType,
                   BootOptions[Index].Attributes,
                   BootOptions[Index].Description,
                   BootOptions[Index].FilePath,
                   BootOptions[Index].OptionalData,
                   BootOptions[Index].OptionalDataSize
                   );
        ASSERT_EFI_ERROR (Status);
        break;
      }
    }
  }

  EfiBootManagerFreeLoadOptions (BootOptions, BootOptionCount);

  //
  // Automatically create the Boot#### for Boot Manager Menu when not found.
  //
  if (Index == BootOptionCount) {
    return RegisterBootManagerMenu (BootOption);
  } else {
    return EFI_SUCCESS;
  }
}


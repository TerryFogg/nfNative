/*
//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) 2006..2015 Giovanni Di Sirio.  All rights reserved.
// See LICENSE file in the project root for full license information.
//
*/

/* Stack rules inclusion.*/
INCLUDE Platform\Memory\rules_stack.ld

/* Code rules inclusion.*/
INCLUDE Platform\Memory\rules_code.ld

/* Data rules inclusion.*/
INCLUDE Platform\Memory\rules_data.ld

/* Memory rules inclusion.*/
INCLUDE Platform\Memory\rules_memory.ld

/* nanoCLR rules inclusion.*/
INCLUDE Platform\Memory\rules_clr.ld

/* boot clipboard rules inclusion.*/
INCLUDE Platform\Memory\rules_bootclipboard.ld


INCLUDE Platform\Memory\rules_graphics.ld
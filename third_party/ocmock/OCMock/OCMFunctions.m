/*
 *  Copyright (c) 2014-2015 Erik Doernenburg and contributors
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use these files except in compliance with the License. You may obtain
 *  a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 */

#import <objc/runtime.h>
#import "OCMFunctions.h"
#import "OCMLocation.h"
#import "OCClassMockObject.h"
#import "OCPartialMockObject.h"


#pragma mark  Known private API

@interface NSException(OCMKnownExceptionMethods)
+ (NSException *)failureInFile:(NSString *)file atLine:(int)line withDescription:(NSString *)formatString, ...;
@end

@interface NSObject(OCMKnownTestCaseMethods)
- (void)recordFailureWithDescription:(NSString *)description inFile:(NSString *)file atLine:(NSUInteger)line expected:(BOOL)expected;
- (void)failWithException:(NSException *)exception;
@end


#pragma mark  Functions related to ObjC type system

BOOL OCMIsObjectType(const char *objCType)
{
    objCType = OCMTypeWithoutQualifiers(objCType);

    if(strcmp(objCType, @encode(id)) == 0 || strcmp(objCType, @encode(Class)) == 0)
        return YES;

    // if the returnType is a typedef to an object, it has the form ^{OriginClass=#}
    NSString *regexString = @"^\\^\\{(.*)=#.*\\}";
    NSRegularExpression *regex = [NSRegularExpression regularExpressionWithPattern:regexString options:0 error:NULL];
    NSString *type = [NSString stringWithCString:objCType encoding:NSASCIIStringEncoding];
    if([regex numberOfMatchesInString:type options:0 range:NSMakeRange(0, type.length)] > 0)
        return YES;

    // if the return type is a block we treat it like an object
    // TODO: if the runtime were to encode the block's argument and/or return types, this test would not be sufficient
    if(strcmp(objCType, @encode(void(^)())) == 0)
        return YES;

    return NO;
}


const char *OCMTypeWithoutQualifiers(const char *objCType)
{
    while(strchr("rnNoORV", objCType[0]) != NULL)
        objCType += 1;
    return objCType;
}


static BOOL ParseStructType(const char *type, const char **typeEnd, const char **typeNameEnd, const char **typeEqualSign)
{
  if (type[0] != '{' && type[0] != '(')
      return NO;

  *typeNameEnd = NULL;
  *typeEqualSign = NULL;

  const char endChar = type[0] == '{' ? '}' : ')';
  for (const char* ptr = type + 1; *ptr; ++ptr) {
      switch (*ptr) {
          case '(':
          case '{':
          {
              const char *subTypeEnd;
              const char *subTypeNameEnd;
              const char *subTypeEqualSign;
              if (!ParseStructType(ptr, &subTypeEnd, &subTypeNameEnd, &subTypeEqualSign))
                  return NO;
              ptr = subTypeEnd;
              break;
          }
          case '=':
          {
              if (!*typeEqualSign) {
                  *typeNameEnd = ptr;
                  *typeEqualSign = ptr;
              }
              break;
          }
          case ')':
          case '}':
          {
              if (*ptr == endChar) {
                  *typeEnd = ptr;
                  if (!*typeNameEnd)
                      *typeNameEnd = ptr;
                  return YES;
              }
              break;
          }
          default:
              break;
      }
  }

  return NO;
}


/*
 * Sometimes an external type is an opaque struct (which will have an @encode of "{structName}"
 * or "{structName=}") but the actual method return type, or property type, will know the contents
 * of the struct (so will have an objcType of say "{structName=iiSS}".  This function will determine
 * those are equal provided they have the same structure name, otherwise everything else will be
 * compared textually.  This can happen particularly for pointers to such structures, which still
 * encode what is being pointed to.
 *
 * In addition, this funtion will consider structures with unknown names, encoded as "{?=}, equal to
 * structures with any name. This means that "{?=dd}" and "{foo=dd}", and even "{?=}" and "{foo=dd}",
 * are considered equal.
 *
 * For some types some runtime functions throw exceptions, which is why we wrap this in an
 * exception handler just below.
 */
static BOOL OCMEqualTypesAllowingOpaqueStructsInternal(const char *type1, const char *type2)
{
    type1 = OCMTypeWithoutQualifiers(type1);
    type2 = OCMTypeWithoutQualifiers(type2);

    switch (type1[0])
    {
        case '{':
        case '(':
        {
            if (type2[0] != type1[0])
                return NO;

            const char *type1End;
            const char *type1NameEnd;
            const char *type1EqualSign;
            if (!ParseStructType(type1, &type1End, &type1NameEnd, &type1EqualSign))
                return NO;

            const char *type2End;
            const char *type2NameEnd;
            const char *type2EqualSign;
            if (!ParseStructType(type2, &type2End, &type2NameEnd, &type2EqualSign))
                return NO;

            /* Opaque types either don't have an equals sign (just the name and the end brace), or
             * empty content after the equals sign.
             * We want that to compare the same as a type of the same name but with the content.
             */
            BOOL type1Opaque = (type1EqualSign == NULL || type1EqualSign + 1 == type1End);
            BOOL type2Opaque = (type2EqualSign == NULL || type2EqualSign + 2 == type2End);
            intptr_t type1NameLen = type1NameEnd - type1;
            intptr_t type2NameLen = type2NameEnd - type2;

            /* If the names are not equal and neither of the names is a question mark, return NO */
            if ((type1NameLen != type2NameLen || strncmp(type1, type2, type1NameLen)) &&
                !((type1NameLen == 2) && (type1[1] == '?')) && !((type2NameLen == 2) && (type2[1] == '?')) &&
                !(type1NameLen == 1 || type2NameLen == 1))
                return NO;

            /* If the same name, and at least one is opaque, that is close enough. */
            if (type1Opaque || type2Opaque)
                return YES;

            /* Otherwise, compare all the elements.  Use NSGetSizeAndAlignment to walk through the struct elements. */
            type1 = type1EqualSign + 1;
            type2 = type2EqualSign + 1;
            while (type1 != type1End && *type1)
            {
                if (!OCMEqualTypesAllowingOpaqueStructs(type1, type2))
                    return NO;

                if (*type1 != '{' && *type1 != '(') {
                    type1 = NSGetSizeAndAlignment(type1, NULL, NULL);
                    type2 = NSGetSizeAndAlignment(type2, NULL, NULL);
                } else {
                    const char *subType1End;
                    const char *subType1NameEnd;
                    const char *subType1EqualSign;
                    if (!ParseStructType(type1, &subType1End, &subType1NameEnd, &subType1EqualSign))
                        return NO;

                    const char *subType2End;
                    const char *subType2NameEnd;
                    const char *subType2EqualSign;
                    if (!ParseStructType(type2, &subType2End, &subType2NameEnd, &subType2EqualSign))
                        return NO;

                    type1 = subType1End + 1;
                    type2 = subType2End + 1;
                }
            }
            return YES;
        }
        case '^':
            /* for a pointer, make sure the other is a pointer, then recursively compare the rest */
            if (type2[0] != type1[0])
                return NO;
            return OCMEqualTypesAllowingOpaqueStructs(type1 + 1, type2 + 1);

        case '?':
            return type2[0] == '?';

        case '\0':
            return type2[0] == '\0';

        default:
        {
            // Move the type pointers past the current types, then compare that region
            const char *afterType1 =  NSGetSizeAndAlignment(type1, NULL, NULL);
            const char *afterType2 =  NSGetSizeAndAlignment(type2, NULL, NULL);
            intptr_t type1Len = afterType1 - type1;
            intptr_t type2Len = afterType2 - type2;

            return (type1Len == type2Len && (strncmp(type1, type2, type1Len) == 0));
        }
    }
}

BOOL OCMEqualTypesAllowingOpaqueStructs(const char *type1, const char *type2)
{
    @try
    {
        return OCMEqualTypesAllowingOpaqueStructsInternal(type1, type2);
    }
    @catch (NSException *e)
    {
        /* Probably a bitfield or something that NSGetSizeAndAlignment chokes on, oh well */
        return NO;
    }
}


#pragma mark  Creating classes

Class OCMCreateSubclass(Class class, void *ref)
{
    const char *className = [[NSString stringWithFormat:@"%@-%p-%u", NSStringFromClass(class), ref, arc4random()] UTF8String];
    Class subclass = objc_allocateClassPair(class, className, 0);
    objc_registerClassPair(subclass);
    return subclass;
}


#pragma mark  Directly manipulating the isa pointer (look away)

void OCMSetIsa(id object, Class class)
{
    *((Class *)object) = class;
}

Class OCMGetIsa(id object)
{
    return *((Class *)object);
}


#pragma mark  Alias for renaming real methods

static NSString *const OCMRealMethodAliasPrefix = @"ocmock_replaced_";
static const char *const OCMRealMethodAliasPrefixCString = "ocmock_replaced_";

BOOL OCMIsAliasSelector(SEL selector)
{
    return [NSStringFromSelector(selector) hasPrefix:OCMRealMethodAliasPrefix];
}

SEL OCMAliasForOriginalSelector(SEL selector)
{
    char aliasName[2048];
    const char *originalName = sel_getName(selector);
    strlcpy(aliasName, OCMRealMethodAliasPrefixCString, sizeof(aliasName));
    strlcat(aliasName, originalName, sizeof(aliasName));
    return sel_registerName(aliasName);
}

SEL OCMOriginalSelectorForAlias(SEL selector)
{
    if(!OCMIsAliasSelector(selector))
        [NSException raise:NSInvalidArgumentException format:@"Not an alias selector; found %@", NSStringFromSelector(selector)];
    NSString *string = NSStringFromSelector(selector);
    return NSSelectorFromString([string substringFromIndex:[OCMRealMethodAliasPrefix length]]);
}


#pragma mark  Wrappers around associative references

static NSString *const OCMClassMethodMockObjectKey = @"OCMClassMethodMockObjectKey";

void OCMSetAssociatedMockForClass(OCClassMockObject *mock, Class aClass)
{
    if((mock != nil) && (objc_getAssociatedObject(aClass, OCMClassMethodMockObjectKey) != nil))
        [NSException raise:NSInternalInconsistencyException format:@"Another mock is already associated with class %@", NSStringFromClass(aClass)];
    objc_setAssociatedObject(aClass, OCMClassMethodMockObjectKey, mock, OBJC_ASSOCIATION_ASSIGN);
}

OCClassMockObject *OCMGetAssociatedMockForClass(Class aClass, BOOL includeSuperclasses)
{
    OCClassMockObject *mock = nil;
    do
    {
        mock = objc_getAssociatedObject(aClass, OCMClassMethodMockObjectKey);
        aClass = class_getSuperclass(aClass);
    }
    while((mock == nil) && (aClass != nil) && includeSuperclasses);
    return mock;
}

static NSString *const OCMPartialMockObjectKey = @"OCMPartialMockObjectKey";

void OCMSetAssociatedMockForObject(OCClassMockObject *mock, id anObject)
{
    if((mock != nil) && (objc_getAssociatedObject(anObject, OCMPartialMockObjectKey) != nil))
        [NSException raise:NSInternalInconsistencyException format:@"Another mock is already associated with object %@", anObject];
    objc_setAssociatedObject(anObject, OCMPartialMockObjectKey, mock, OBJC_ASSOCIATION_ASSIGN);
}

OCPartialMockObject *OCMGetAssociatedMockForObject(id anObject)
{
    return objc_getAssociatedObject(anObject, OCMPartialMockObjectKey);
}


#pragma mark  Functions related to IDE error reporting

void OCMReportFailure(OCMLocation *loc, NSString *description)
{
    id testCase = [loc testCase];
    if((testCase != nil) && [testCase respondsToSelector:@selector(recordFailureWithDescription:inFile:atLine:expected:)])
    {
        [testCase recordFailureWithDescription:description inFile:[loc file] atLine:[loc line] expected:NO];
    }
    else if((testCase != nil) && [testCase respondsToSelector:@selector(failWithException:)])
    {
        NSException *exception = nil;
        if([NSException instancesRespondToSelector:@selector(failureInFile:atLine:withDescription:)])
        {
            exception = [NSException failureInFile:[loc file] atLine:(int)[loc line] withDescription:description];
        }
        else
        {
            NSString *reason = [NSString stringWithFormat:@"%@:%lu %@", [loc file], (unsigned long)[loc line], description];
            exception = [NSException exceptionWithName:@"OCMockTestFailure" reason:reason userInfo:nil];
        }
        [testCase failWithException:exception];
    }
    else if(loc != nil)
    {
        NSLog(@"%@:%lu %@", [loc file], (unsigned long)[loc line], description);
        NSString *reason = [NSString stringWithFormat:@"%@:%lu %@", [loc file], (unsigned long)[loc line], description];
        [[NSException exceptionWithName:@"OCMockTestFailure" reason:reason userInfo:nil] raise];

    }
    else
    {
        NSLog(@"%@", description);
        [[NSException exceptionWithName:@"OCMockTestFailure" reason:description userInfo:nil] raise];
    }

}
